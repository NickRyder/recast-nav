# Benchmarks

The benchmark tools compare the Rust implementation with the pinned,
unmodified RecastNavigation C++ sources. Measure the source and configuration
you intend to use; results from an older revision do not establish current
performance. The repository does not promise a speedup or lower memory use for
every workload.

## Run on macOS

The native worker requires a GNU-compatible C++ driver. Use Clang to reproduce
the separate-operation arithmetic policy. Benchmarks use 32-bit references;
leave `fma` and `polyref64` disabled.

```sh
CXX=clang++ cargo build --locked --release -p recast-nav --features native-bench --example library_bench
```

Prepare fixtures, then measure the already-built workers on an idle machine.
For a complete acceptance run with allocation profiling:

```sh
target/release/examples/library_bench prepare target/bench-fixtures
native_worker=$(target/release/examples/library_bench native-path target/bench-fixtures)
python3 scripts/bench_libraries.py \
  --rust target/release/examples/library_bench --native "$native_worker" \
  --fixtures target/bench-fixtures --output target/bench-results.json \
  --profiler "$(dirname "$native_worker")/library-allocations.dylib" \
  --calibration "$(dirname "$native_worker")/allocation-calibration" \
  --repetitions 12 --target-ms 1000
python3 scripts/check_performance_targets.py target/bench-results.json
```

This command uses twelve balanced repetitions with a 1000 ms calibration target.
It checks complete output checksums before timing, records worker and
fixture hashes, and calibrates the macOS allocation interposer. A calibration
target is not a guaranteed minimum duration for each timed sample. See
`python3 scripts/bench_libraries.py --help` for focused cases and alternate sample
targets. The worker's shorter `measure` command uses seven repetitions and a
250 ms target for diagnostics; it does not meet the acceptance policy.

The acceptance checker requires every workload to have Rust median runtime and
cumulative requested allocation bytes at or below C++. It reports failures
without averaging them away. A passing report applies only to its recorded
source, binaries, fixtures, compiler policies, and platform.

## Workloads

The 27 workloads cover Recast generation and partitioning strategies, stacked
geometry, utility operations, Detour queries and lifecycle operations, TileCache
construction/streaming/codecs, crowd simulation, path queues, proximity grids,
and obstacle avoidance. The worker can run one case directly:

```sh
target/release/examples/library_bench tilecache-builder target/bench-fixtures 1000
```

Keep an exclusive machine reservation for timing when a local build coordinator
is available. Build before taking that reservation, and do not compile or run
profilers concurrently with timing samples.

## Measurement boundaries

Both workers use identical fixtures, output budgets, and full-output checksums.
Detour queries retain equivalent caller-owned output buffers between episodes.
TileCache snapshots borrow tile bodies in both languages. Construction, scratch
setup, hashing, copies, and teardown stay inside the timer when they are part of
the workload. Fixture preparation, input loading, and persistent query setup are
excluded in both implementations.

Allocation bytes are cumulative successful main-thread heap requests, including
reallocations. The profiler measures the difference between three and six
episodes, with three identical repetitions. These bytes exclude stack storage
and are not peak live memory. Timing samples do not load the allocation
interposer. Peak RSS includes process startup and retained state.

Near-threshold timings need independent confirmation. Preserve failed probes,
raw samples, and source identifiers while investigating regressions. Sampled CPU
shares identify candidates for investigation; they do not establish causal time
savings. See the [optimization workflow](optimization.md).
