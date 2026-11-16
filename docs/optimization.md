# Optimization workflow

Preserve native-defined numeric ordering, statuses, callbacks, retained state,
serialized bytes, and initialization contracts. Keep application policies outside
native algorithms and leave the reference sources unchanged.

1. Profile the exact workload and frozen executable that needs improvement.
   Use matched phase timing when a sampled symbol does not explain the cost.
2. Change one concrete cost at a time. Keep fixtures, compiler settings, reusable
   buffers, hashing, copies, ownership, and measured boundaries equivalent.
3. Run a focused native differential regression before measuring a candidate.
4. Repeat narrow timing wins independently and check allocation traffic. Keep
   negative results and source/executable hashes; remove unsuccessful experiments.
5. Validate changed behavior across the supported arithmetic/reference modes,
   review the complete change, and check the full workload set for regressions.

Passing performance targets does not establish behavioral parity. Report
remaining mismatches, coverage gaps, regressions, and platform limitations.
See [benchmarks](performance.md), [conformance](conformance.md), and the
[coverage ledger](coverage.md).
