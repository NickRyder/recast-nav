"""Contract checks for the per-workload acceptance gate; no benchmark timings."""
import copy
import hashlib
import unittest

from bench_libraries import CASES
from check_performance_targets import check


def report():
    value = {"schema_version": 3, "head": "a" * 40, "source_origin": "git",
             "source_diff_sha256": hashlib.sha256(b"").hexdigest(), "platform": "macOS-test",
             "rust_policy": "release opt-level=3; default codegen units; no LTO; fma/polyref64 disabled",
             "native_policy": "-O3 -fno-fast-math -ffp-contract=off -fno-builtin-sinf -fno-builtin-cosf; unmodified pinned library translation units",
             "tilecache_snapshot_policy": "borrowed-tile-bytes-v1",
             "measurement_policy": {"repetitions": 12, "target_ms": 1000, "validate_only": False},
             "allocation_calibration": {"delta": {"calls": 600, "requested_bytes": 66800, "frees": 500}},
             "workloads": {}}
    for case in CASES:
        row = {"iterations_per_sample": 1000,
               "sample_orders": [["rust", "native"], ["native", "rust"]] * 6,
               "validation": {}, "samples": {}, "median_ns_per_episode": {}, "allocations": {}}
        for role, elapsed in [("rust", 900_000_000), ("native", 1_000_000_000)]:
            row["validation"][role] = {"case": case, "iterations": 1, "checksum": "abc"}
            row["samples"][role] = [{"case": case, "iterations": 1000, "elapsed_ns": elapsed, "checksum": "def"} for _ in range(12)]
            row["median_ns_per_episode"][role] = elapsed / 1000
            counts = {"calls": 100 if role == "rust" else 5, "requested_bytes": 100, "frees": 5}
            row["allocations"][role] = {"per_episode": counts, "samples": [copy.copy(counts) for _ in range(3)]}
        value["workloads"][case] = row
    return value


class PerformanceTargets(unittest.TestCase):
    def test_each_metric_is_required_even_when_every_other_case_is_fast(self):
        for metric in ("speed", "bytes"):
            data = report()
            row = data["workloads"][CASES[0]]
            if metric == "speed":
                for sample in row["samples"]["rust"]:
                    sample["elapsed_ns"] = 1_000_001_000
                row["median_ns_per_episode"]["rust"] = 1_000_001
            else:
                row["allocations"]["rust"]["per_episode"]["requested_bytes"] = 101
                for sample in row["allocations"]["rust"]["samples"]:
                    sample["requested_bytes"] = 101
            result = check(data, False)
            self.assertFalse(result["accepted"])
            self.assertEqual(result["workloads_meeting_both"], len(CASES) - 1)

    def test_equal_bytes_and_zero_bytes_pass_without_call_count_substitution(self):
        data = report()
        row = data["workloads"][CASES[0]]
        row["samples"]["rust"] = copy.deepcopy(row["samples"]["native"])
        row["median_ns_per_episode"]["rust"] = row["median_ns_per_episode"]["native"]
        for role in ("rust", "native"):
            row["allocations"][role]["per_episode"]["requested_bytes"] = 0
            for sample in row["allocations"][role]["samples"]:
                sample["requested_bytes"] = 0
        self.assertTrue(check(data, False)["accepted"])
        row["allocations"]["rust"]["per_episode"]["requested_bytes"] = 1
        for sample in row["allocations"]["rust"]["samples"]:
            sample["requested_bytes"] = 1
        self.assertFalse(check(data, False)["accepted"])

    def test_partial_short_unbalanced_and_diagnostic_runs_cannot_claim_acceptance(self):
        for alteration in ("missing", "short", "unbalanced", "missing_orders", "snapshot_ownership", "diagnostic"):
            data = report()
            if alteration == "missing":
                del data["workloads"][CASES[0]]
            elif alteration == "short":
                data["measurement_policy"]["target_ms"] = 750
            elif alteration == "unbalanced":
                data["workloads"][CASES[0]]["sample_orders"] = [["rust", "native"]] * 12
            elif alteration == "snapshot_ownership":
                del data["tilecache_snapshot_policy"]
            elif alteration == "missing_orders":
                data["workloads"][CASES[0]]["sample_orders"] = [["rust", "native"], ["native", "rust"]]
            self.assertFalse(check(data, alteration == "diagnostic")["accepted"])

    def test_inconsistent_raw_evidence_is_rejected(self):
        for alteration in ("median", "checksum", "workload", "allocation"):
            data = report()
            row = data["workloads"][CASES[0]]
            if alteration == "median":
                row["median_ns_per_episode"]["rust"] = 1
            elif alteration == "checksum":
                row["samples"]["rust"][0]["checksum"] = "different"
            elif alteration == "workload":
                row["samples"]["rust"][0]["case"] = "different"
            else:
                row["allocations"]["rust"]["samples"][0]["requested_bytes"] += 1
            with self.assertRaises(ValueError):
                check(data, False)


if __name__ == "__main__":
    unittest.main()
