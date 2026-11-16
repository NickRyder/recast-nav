#!/usr/bin/env python3
"""Re-extract pinned public C++ declarations and check the reviewed Rust map.

Clang is an audit-time dependency only. The map documents API adaptations; it
does not establish behavioral equivalence or validate Rust target expressions.
"""

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]
MAP = ROOT / "docs/public-api.json"
LIBRARIES = ("Recast", "Detour", "DetourTileCache", "DetourCrowd")
CALLABLES = {
    "FunctionDecl", "CXXMethodDecl", "CXXConstructorDecl",
    "CXXDestructorDecl", "CXXConversionDecl",
}
RECORDS = {"CXXRecordDecl", "RecordDecl"}
SPECIALIZATIONS = {
    "ClassTemplateSpecializationDecl", "ClassTemplatePartialSpecializationDecl",
}


def extract(clang, polyref64=False):
    headers = {
        str(p.relative_to(ROOT)): p.read_bytes()
        for library in LIBRARIES
        for p in sorted((ROOT / "vendor/recastnavigation" / library / "Include").glob("*.h"))
    }
    with tempfile.TemporaryDirectory(prefix="recast-api-") as directory:
        driver = Path(directory) / "api.cpp"
        driver.write_text("".join(f'#include "{name}"\n' for name in headers))
        command = [clang, "-std=c++11", "-DDT_VIRTUAL_QUERYFILTER=1", "-I", str(ROOT)]
        if polyref64:
            command.append("-DDT_POLYREF64=1")
        for library in LIBRARIES:
            command += ["-I", str(ROOT / "vendor/recastnavigation" / library / "Include")]
        command += ["-fsyntax-only", "-Xclang", "-ast-dump=json", str(driver)]
        # Keep the potentially large AST out of the terminal and repository.
        with (Path(directory) / "ast.json").open("w+") as output:
            subprocess.run(command, stdout=output, check=True)
            output.seek(0)
            ast = json.load(output)

    ids = {}

    def index(node):
        if "id" in node:
            ids[node["id"]] = node
        for child in node.get("inner", []):
            index(child)

    index(ast)

    def source(node, hint):
        loc = node.get("loc", {})
        loc = loc.get("spellingLoc", loc)
        offset = loc.get("offset")
        name = node.get("name", "").split("<", 1)[0]
        if offset is None or not name:
            return None
        token = "operator" if name.startswith("operator") else name
        matches = [
            path for path, data in headers.items()
            if data[offset:offset + len(token)] == token.encode()
            and (token == "operator" or loc.get("tokLen") == len(token)
                 or token.startswith("~"))
        ]
        if len(matches) != 1:
            matches = [hint] if hint in matches else []
        if not matches:
            return None
        path = matches[0]
        return {"file": path, "line": headers[path][:offset].count(b"\n") + 1, "offset": offset}

    calls, fields, constants, aliases, unresolved = [], [], [], [], []

    def walk(node, contexts=(), access="public", visible=True, hint=None):
        kind, name = node.get("kind"), node.get("name", "")
        if kind in SPECIALIZATIONS or node.get("isImplicit"):
            return
        if kind in RECORDS:
            visible = visible and access == "public"
            if name:
                contexts += (name,)
            location = source(node, hint)
            if location:
                hint = location["file"]
            access = "private" if node.get("tagUsed") == "class" else "public"
        native = name.startswith(("rc", "dt")) or any(
            context.startswith(("rc", "dt")) for context in contexts
        ) or "rcNewTag" in node.get("type", {}).get("qualType", "")
        if kind in CALLABLES:
            if not contexts and node.get("parentDeclContextId"):
                contexts = (ids.get(node["parentDeclContextId"], {}).get("name", ""),)
                native = native or any(c.startswith(("rc", "dt")) for c in contexts)
            if native and visible and access != "private" and not node.get("previousDecl"):
                location = source(node, hint)
                if location:
                    calls.append({
                        **location, "name": "::".join(contexts + (name,)),
                        "signature": node.get("type", {}).get("qualType", ""),
                        "access": access, "virtual": node.get("virtual", False), "kind": kind,
                    })
                else:
                    unresolved.append("::".join(contexts + (name,)))
            return
        if kind == "FieldDecl" and name and native and visible and access == "public":
            location = source(node, hint)
            if location:
                fields.append({
                    **location, "owner": "::".join(contexts), "name": name,
                    "type": node.get("type", {}).get("qualType", ""),
                    "bitfield": node.get("isBitfield", False),
                })
            else:
                unresolved.append("::".join(contexts + (name,)))
        if visible and access == "public" and (
            (kind in {"EnumConstantDecl", "VarDecl"} and name.startswith(("RC_", "DT_")))
            or (kind == "TypedefDecl" and native)
        ):
            location = source(node, hint)
            if location:
                target = aliases if kind == "TypedefDecl" else constants
                target.append({
                    **location, "name": "::".join(contexts + (name,)), "kind": kind,
                    "type": node.get("type", {}).get("qualType", ""),
                })
            else:
                unresolved.append(name)
        for child in node.get("inner", []):
            if child.get("kind") == "AccessSpecDecl":
                access = child["access"]
            else:
                walk(child, contexts, access, visible, hint)

    walk(ast)
    if unresolved:
        raise SystemExit(f"Unresolved source anchors: {unresolved}")

    def unique(records):
        keyed = {}
        for record in records:
            # Clang also emits instantiated template functions at the same
            # physical location. Retain the primary declaration, visited first.
            keyed.setdefault((record["file"], record["offset"]), record)
        return sorted(keyed.values(), key=lambda r: (r["file"], r["offset"]))

    return {
        "headers": {name: hashlib.sha256(data).hexdigest() for name, data in sorted(headers.items())},
        "callables": unique(calls), "fields": unique(fields),
        "constants": unique(constants), "type_aliases": unique(aliases),
    }


def inventory(clang):
    narrow, wide = extract(clang), extract(clang, polyref64=True)
    for group in ("headers", "callables", "fields"):
        if narrow[group] != wide[group]:
            raise SystemExit(f"Reference-width-dependent {group} need explicit review")
    for group in ("constants", "type_aliases"):
        merged = {}
        for configuration, records in (("polyref32", narrow[group]), ("polyref64", wide[group])):
            for record in records:
                key = json.dumps(record, sort_keys=True)
                if key not in merged:
                    merged[key] = {**record, "configurations": []}
                merged[key]["configurations"].append(configuration)
        narrow[group] = sorted(merged.values(), key=lambda r: (r["file"], r["offset"]))
    # Capture all definitions of the operational macros, including NDEBUG and
    # compiler branches. Include guards and external build switches are not APIs.
    macros = []
    for path in narrow["headers"]:
        offset = 0
        for line_number, line in enumerate((ROOT / path).read_bytes().splitlines(keepends=True), 1):
            match = re.match(rb"#\s*define\s+((?:rc|dt|RC_)\w+)", line)
            if match:
                macros.append({"file": path, "line": line_number,
                               "offset": offset + match.start(1),
                               "name": match.group(1).decode()})
            offset += len(line)
    narrow["macros"] = macros
    return narrow


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--clang", default="clang++")
    args = parser.parse_args()
    actual = inventory(args.clang)
    expected = json.loads(MAP.read_text())
    if actual["headers"] != expected["headers"]:
        raise SystemExit("Pinned public header hashes differ from the reviewed map")
    for group, extra in (
        ("callables", {"rust", "classification", "qualification"}),
        ("fields", {"rust_type", "rust_member", "classification"}),
        ("constants", {"rust", "classification", "qualification"}),
        ("type_aliases", {"rust", "classification", "qualification"}),
        ("macros", {"rust", "classification", "qualification"}),
    ):
        declarations = [{key: value for key, value in item.items() if key not in extra}
                        for item in expected[group]]
        if actual[group] != declarations:
            with tempfile.NamedTemporaryFile(mode="w", prefix=f"recast-{group}-", suffix=".json", delete=False) as output:
                json.dump(actual[group], output, indent=2)
            raise SystemExit(f"Public {group} differ; extracted declarations: {output.name}")
        if any(not item.get("classification") or not item.get("rust", item.get("rust_member"))
               for item in expected[group]):
            raise SystemExit(f"Missing reviewed mapping in {group}")
    print(f"Pinned API map: {len(actual['headers'])} headers, "
          f"{len(actual['callables'])} callable declarations, {len(actual['fields'])} public fields, "
          f"{len(actual['constants'])} constants, {len(actual['type_aliases'])} type-alias declarations, "
          f"{len(actual['macros'])} macro definitions (both reference widths)")


if __name__ == "__main__":
    main()
