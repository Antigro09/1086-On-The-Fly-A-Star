# Executed development commands

JDK17.0.20.1+1, release17 common classes, fixed128MiB heap. All planning runs used one worker and ran sequentially.

```sh
javac --release 17 -Xlint:all -Werror -d build/benchmark-classes \
 vendor/world-state-api-src/org/frcworldstate/core/*.java \
 src/main/java/pathplanning/geometric/*.java \
 src/worldStateAdapter/java/pathplanning/backend/*.java \
 src/benchmark/java/pathplanning/benchmark/*.java
java -Xms128m -Xmx128m -cp build/benchmark-classes \
 pathplanning.benchmark.PlannerBenchmark --warmup 100 --samples 500 \
 --out build/benchmarks/baseline-jdk17-adapter
```

Repeated with separate `--phase core`, a second fresh-JVM adapter run, then after the empty-static-grid change with output `optimized-jdk17-adapter`, `optimized-jdk17-core` and `optimized-jdk17-repeat`. Also executed separately:

```sh
java -Xms128m -Xmx128m -cp build/benchmark-classes \
 pathplanning.benchmark.PlannerBenchmark --warmup 100 --samples 500 \
 --timeout-ms 1 --deadline-ms 1 --out build/benchmarks/optimized-jdk17-1ms-budget
java -Xms128m -Xmx128m -cp build/benchmark-classes \
 pathplanning.benchmark.PlannerBenchmark --warmup 50 --samples 200 \
 --width-m 16 --height-m 8 --out build/benchmarks/optimized-jdk17-large
```

Temurin25.0.4.1+1 repeated the common17bytecode adapter run with100warm-up/500samples, sameheap and output `optimized-jdk25-adapter`.

Before optimization, a separate JDK17 run recorded JFR:

```sh
java -XX:StartFlightRecording=filename=build/benchmarks/baseline-profile.jfr,settings=profile,dumponexit=true,maxsize=32m \
 -Xms128m -Xmx128m -cp build/benchmark-classes \
 pathplanning.benchmark.PlannerBenchmark --warmup 100 --samples 500 \
 --out build/benchmarks/baseline-jdk17-profile
```

`jfr` was absent from shell PATH (exit127); the recording itself succeeded. The explicit JDK17 `bin/jfr print --json --events jdk.ExecutionSample,jdk.ObjectAllocationSample` then successfully exported the existing recording for the sampled summary. No external package/tool installation was required.
