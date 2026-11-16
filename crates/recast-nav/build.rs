fn main() {
    #[cfg(feature = "cpp-oracle")]
    build_oracles();
    #[cfg(feature = "native-bench")]
    build_benchmark();
}
#[cfg(feature = "cpp-oracle")]
fn build_oracles() {
    use std::{env, path::PathBuf};
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    println!(
        "cargo:rerun-if-changed={}",
        root.join("test-support").display()
    );
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let compiler = cc::Build::new().cpp(true).get_compiler();
    assert!(
        !compiler.is_like_msvc(),
        "The conformance runner requires a GNU-compatible C++ driver. Runtime Rust has no native dependency."
    );
    if cfg!(feature = "fma") {
        assert!(
            compiler.is_like_clang(),
            "Use CXX=clang++ for the FMA conformance oracle; contraction policy is compiler-specific."
        );
    }
    let (sources, includes) = native_sources(&root);
    for (source, name, variable) in [
        ("oracle.cpp", "recast-oracle", "RECAST_CPP_ORACLE"),
        (
            "runtime_oracle.cpp",
            "runtime-oracle",
            "RECAST_RUNTIME_ORACLE",
        ),
        (
            "recast_utilities_oracle.cpp",
            "recast-utilities-oracle",
            "RECAST_UTILITIES_ORACLE",
        ),
        (
            "detour_mesh_oracle.cpp",
            "detour-mesh-oracle",
            "DETOUR_MESH_ORACLE",
        ),
        (
            "detour_data_oracle.cpp",
            "detour-data-oracle",
            "DETOUR_DATA_ORACLE",
        ),
        (
            "tile_cache_oracle.cpp",
            "tile-cache-oracle",
            "TILE_CACHE_ORACLE",
        ),
        ("crowd_oracle.cpp", "crowd-oracle", "CROWD_ORACLE"),
    ] {
        let source = root.join("test-support").join(source);
        let executable = native_executable(&out, name);
        let mut command = compiler.to_command();
        command.args([
            native_cpp_standard(),
            "-O0",
            "-fno-fast-math",
            if cfg!(feature = "fma") {
                "-ffp-contract=on"
            } else {
                "-ffp-contract=off"
            },
        ]);
        if cfg!(feature = "fma") && env::var("CARGO_CFG_TARGET_ARCH").as_deref() == Ok("x86_64") {
            command.arg("-mfma");
        }
        if cfg!(feature = "polyref64") {
            command.arg("-DDT_POLYREF64=1");
        }
        if name == "detour-data-oracle" {
            command.arg("-DDT_VIRTUAL_QUERYFILTER=1");
        }
        for include in &includes {
            command.arg("-I").arg(include);
        }
        command
            .arg(source)
            // The cache driver includes this unmodified translation unit to
            // call its private numeric helpers directly as well as its APIs.
            .args(sources.iter().filter(|p| {
                (name != "tile-cache-oracle"
                    || p.file_name().unwrap() != "DetourTileCacheBuilder.cpp")
                    && (name == "crowd-oracle"
                        || !p.components().any(|c| c.as_os_str() == "DetourCrowd"))
            }))
            .arg("-o")
            .arg(&executable);
        assert!(
            command
                .status()
                .expect("start C++ reference compiler")
                .success(),
            "compile {name}"
        );
        println!("cargo:rustc-env={variable}={}", executable.display());
    }
}

#[cfg(feature = "native-bench")]
fn build_benchmark() {
    use std::{env, path::PathBuf};
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let compiler = cc::Build::new().cpp(true).get_compiler();
    assert!(
        !compiler.is_like_msvc(),
        "Native benchmarks need a GNU-compatible driver"
    );
    let rustc = std::process::Command::new(env::var_os("RUSTC").unwrap())
        .arg("--version")
        .output()
        .expect("read Rust compiler version");
    let cxx = compiler
        .to_command()
        .arg("--version")
        .output()
        .expect("read native compiler version");
    for (variable, output) in [("RECAST_BENCH_RUSTC", rustc), ("RECAST_BENCH_CXX", cxx)] {
        assert!(output.status.success());
        println!(
            "cargo:rustc-env={variable}={}",
            String::from_utf8(output.stdout)
                .unwrap()
                .lines()
                .next()
                .unwrap()
        );
    }
    let mut command = compiler.to_command();
    command.args([
        native_cpp_standard(),
        "-O3",
        "-fno-fast-math",
        "-ffp-contract=off",
        "-fno-builtin-sinf",
        "-fno-builtin-cosf",
    ]);
    println!(
        "cargo:rerun-if-changed={}",
        root.join("test-support/library_bench_extended.h").display()
    );
    println!(
        "cargo:rerun-if-changed={}",
        root.join("test-support/library_bench_crowd.h").display()
    );
    let source = root.join("test-support/library_bench.cpp");
    println!("cargo:rerun-if-changed={}", source.display());
    command.arg(source);
    let (sources, includes) = native_sources(&root);
    for include in includes {
        command.arg("-I").arg(include);
    }
    let executable = native_executable(&out, "library-bench-native");
    assert!(
        command
            .args(sources)
            .arg("-o")
            .arg(&executable)
            .status()
            .expect("start native benchmark compiler")
            .success(),
        "compile native benchmark"
    );
    println!(
        "cargo:rustc-env=RECAST_NATIVE_BENCH={}",
        executable.display()
    );
    if env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("macos") {
        for (source, name, variable, dylib) in [
            (
                "library_allocations.cpp",
                "library-allocations.dylib",
                "RECAST_ALLOC_PROFILER",
                true,
            ),
            (
                "allocation_calibration.cpp",
                "allocation-calibration",
                "RECAST_ALLOC_CALIBRATION",
                false,
            ),
        ] {
            let source = root.join("test-support").join(source);
            println!("cargo:rerun-if-changed={}", source.display());
            let output = out.join(name);
            let mut command = compiler.to_command();
            command.args(["-std=c++11", "-O2", "-fno-builtin"]);
            if dylib {
                command.arg("-dynamiclib");
            }
            assert!(
                command
                    .arg(source)
                    .arg("-o")
                    .arg(&output)
                    .status()
                    .expect("start allocation instrumentation compiler")
                    .success(),
                "compile allocation instrumentation"
            );
            println!("cargo:rustc-env={variable}={}", output.display());
        }
    }
}

#[cfg(any(feature = "cpp-oracle", feature = "native-bench"))]
fn native_cpp_standard() -> &'static str {
    // Current MSVC standard-library headers require C++14 even with clang++.
    if std::env::var("CARGO_CFG_TARGET_ENV").as_deref() == Ok("msvc") {
        "-std=c++14"
    } else {
        "-std=c++11"
    }
}

#[cfg(any(feature = "cpp-oracle", feature = "native-bench"))]
fn native_executable(out: &std::path::Path, name: &str) -> std::path::PathBuf {
    let mut executable = out.join(name);
    if std::env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("windows") {
        executable.set_extension("exe");
    }
    executable
}

#[cfg(any(feature = "cpp-oracle", feature = "native-bench"))]
fn native_sources(root: &std::path::Path) -> (Vec<std::path::PathBuf>, Vec<std::path::PathBuf>) {
    let mut sources = Vec::new();
    let mut includes = Vec::new();
    for library in ["Recast", "Detour", "DetourTileCache", "DetourCrowd"] {
        let reference = root.join("vendor/recastnavigation").join(library);
        println!("cargo:rerun-if-changed={}", reference.display());
        includes.push(reference.join("Include"));
        sources.extend(
            std::fs::read_dir(reference.join("Source"))
                .unwrap()
                .map(|e| e.unwrap().path())
                .filter(|p| p.extension().is_some_and(|e| e == "cpp")),
        );
    }
    sources.sort();
    (sources, includes)
}
