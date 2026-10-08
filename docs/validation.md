# Executed validation and remaining checks

Implementation date: 2026-10-08. Baseline `e6e8c73821330a5edb5da02658da2fe32ec8da45`; initial `main` checkout clean; branch `codex/geometric-backend-2026`. Existing default-branch work is preserved.

## Host and tools

`uname -a`: `Darwin <host> 25.6.0 ... RELEASE_ARM64_T6050 arm64`. Gradle reports Mac OS X26.6.2 aarch64. Temurin OpenJDK **17.0.20.1+1** arm64; Gradle **8.5**, revision `28aca86a7180baa17117e0e5ba01d8ea9feca598`. Only JDK17 was globally registered; the follow-up used an explicit verified Temurin **25.0.4.1+1** cache path. Detailed device CPU/memory queries via `sysctl` were unavailable in the sandbox. This is development-host evidence, not a controller benchmark.

## Passed commands

Run from the checkout, with a single Gradle worker and test JVM heap128MiB:

```sh
git status --short --branch
git log -1 --format=fuller
git diff --check
shasum -a 256 vendor/world-state-api-src/org/frcworldstate/core/*.java
mkdir -p build/manual-classes
javac --release 17 -d build/manual-classes \
  vendor/world-state-api-src/org/frcworldstate/core/*.java \
  src/main/java/pathplanning/geometric/*.java \
  src/worldStateAdapter/java/pathplanning/backend/*.java \
  src/contractTest/java/pathplanning/geometric/ContractSuite.java
java -Xmx128m -cp build/manual-classes pathplanning.geometric.ContractSuite
./gradlew --no-daemon --max-workers=1 --gradle-user-home ../.gradle-user \
  build worldStateAdapterJar run historicalDemoClasses
./gradlew --version --gradle-user-home ../.gradle-user
jar tf build/libs/on-the-fly-a-star-0.2.0.jar
jar tf build/libs/on-the-fly-a-star-0.2.0-world-state-adapter.jar
```

Initial manual compilation/suite passed16 groups. Final Gradle build passed **17 acceptance groups** after adding endpoint-speed checks, decimal/static tangency regressions, numeric-overflow rejection and deferred future-revocation completion. `verifyWorldStateApiPin` passed against all three owner source hashes. `historicalDemoClasses` compiled the isolated demo against exact WPILib **2026.2.2** on Java17. No native controller libraries or NT server were started. The normal `test` source set is empty; `check` executes the explicit dependency-free **contractCheck** suite, not an unrun JUnit suite.

The suite covers static occupancy, conservative bounded moving envelopes, narrow passages, invalid/nonfinite maps/footprints/constraints, occupied/out-of-bounds starts/goals, strafe/reverse/heading distinction, in-place/zero-length geometry, corner cutting, thin obstacles, exact lower-x/y and decimal-grid tangency, field/tolerance edges, immutable snapshots, no-path/resource caps, monotonic timeout, cancellation during swept scans, pinned API identity/validity, invalid start linear/angular velocity, expired budget, stationary two-point v1 path, latest pending replacement with an old solver ignoring cancellation, and disable/cancel/task/pose/source/map invalidation. Geometry has no timed-trajectory or physical-task-completion field.

The final CPU smoke example returned `SUCCESS`, six poses around the obstacle, and a final stationary rotation. Example solver elapsed about17.7ms is one synthetic smoke observation, **not a latency distribution or planning-frequency benchmark**. Jar inventory confirmed production contains only geometric core; adapter jar adds `pathplanning.backend` and omits the season demo, WPILib and World-State classes. The source pin and dependency direction are preserved.

## Review fixes

Independent review found exact grid-boundary contact could escape candidate enumeration, including decimal cell division rounding. Candidate ranges now include one adjacent cell on every side before exact capsule checks. Review also found extreme finite coordinates could produceNaN distances; envelope coordinates are bounded, and uncomputable collision arithmetic fails closed. Regressions passed in the final build. Async review found diagnostic future continuations could reenter half-updated revocation bookkeeping; invalidation completion now occurs after state mutation and revocation status is preserved under worker races.

## Unrun or remaining

- No roboRIO, Jetson or Systemcore deployment, hardware actuation or physical success verification. Actual robot code integration remains on hold.
- Pure Java core/SPI compilation, packaging and CPU execution passed on Mac JDK25 for release17 and release25 bytecode; no distinct WPILib adapter is required by this backend. Systemcore image/native-library loading and controller runtime qualification remain unrun. WPILib2026+Systemcore is unsupported in the current official matrix.
- API v1 provides conservative raw circle/envelope radius, uncertainty margin and expiry, but not separate object dimensions/observation age or desired final velocity. World-State owns constructing the full-horizon envelope and observation retention; these cannot be reconstructed from an omitted obstacle here. The adapter preserves unknown age rather than fabricating zero.
- No time parameterization, PathPlanner conversion/following, drivetrain speed/feedforward/module-order validation, timed collision checks or execution guard integration. Nonzero speed metadata is validated but geometric search proves no speed/acceleration feasibility.
- No target latency distribution, deadline-miss rate or measured memory impact. Use the separately documented [unrun benchmark procedure](benchmark-procedure.md) when that target work is authorized.

No hardware operation or large training was performed.

## Java25 and direct World-State integration follow-up

Strict JDK17 and JDK25 builds passed all **17 acceptance groups** and **20 owner-envelope/adapter/owner-validator checks**. The JDK25 runs packaged and executed both release17 and release25 bytecode; production dependencies are only `java.base`, the core and World-State's API. The final JDK17 Gradle command `./gradlew --offline --no-daemon --max-workers=1 --gradle-user-home ../.gradle-user build worldStateAdapterJar run` also passed both suites and the smoke example. [Follow-up validation](jdk25-validation.md) records exact profiles, commands, owner commit/hashes and raw results. It supersedes any earlier Java25-unrun status.

## Local desktop and measured optimization follow-up

The separate local branch `codex/mac-benchmark-browser-plan` adds a bounded
desktop dashboard and opt-in benchmark harness. Robot integration was not
performed. Device identification and bounded warmed measurements now appear
in [Mac performance](mac-performance.md), with a published portable command
transcript and locally retained, unpublished raw CSV/JSON under ignored
`docs/evidence/mac-benchmark/`. The measured immutable empty-static-grid shortcut
retains dynamic envelopes, field edges and cancellation. Its regression raises
the current acceptance count to **18**; prior 17-group evidence remains historical.

The current JDK17 Gradle build passed 18 acceptance groups, 20 direct owner-envelope/
adapter/owner-validator checks, 30 desktop model checks and 12 loopback HTTP checks:

```sh
./gradlew --offline --no-daemon --max-workers=1 --gradle-user-home ../.gradle-user \
  build worldStateAdapterJar dashboardContractCheck dashboardHttpCheck
node fixtures/field-map/test-field-map.mjs
node fixtures/dashboard/test-scenario-import.mjs
node --check src/dashboard/resources/dashboard/app.js
node --check src/dashboard/resources/dashboard/field-import.js
```

Node 24.14.0 passed **58 field-map checks**, eight actual-handler async import
regressions and both syntax checks. Strict standalone
Java17 compilation with `-Xlint:all -Werror` passed for the core/adapter/benchmark;
desktop Java17 `-Xlint:all` compilation produced no warnings. Production jar
inventories contain no dashboard, Jackson, historical demo or HTTP-server classes.

The current common release-17 artifact was also freshly compiled/packaged and
executed on the explicitly selected Temurin25.0.4.1+1 using
`python3 tools/verify_jdk25.py --java-home "$JDK25_HOME" --release 17`.
All 18 groups, 20 owner checks, the smoke example, bytecode and dependency checks
passed. Current source hashes and results are retained in
the locally retained, unpublished `docs/evidence/dashboard/jdk25-current.json`.
The earlier separate release-25
bytecode report covers its earlier 17-group source; no native/controller claim
follows from either host check. Review caught the verification script's stale
17-group assertion; it now requires the current 18-group summary and rejects
desktop/Jackson/HTTP dependencies in production artifacts.

Actual local Chromium checks and screenshots are described in
[desktop browser evidence](dashboard-validation.md). These include a
real backend proposal in Sandbox, synthetic replay and mock-only Live, plus
reviewed affine image-map import. Custom-Vision independently consumed the same
golden map/raster/digest in its local field viewer and reported 18 scene checks
and 34 actual Chromium checks; its controller/perception behavior was unchanged.

Real NetworkTables telemetry, official field geometry/calibration accuracy,
drivetrain simulation, controller/native loading, combined robot-stack latency,
hardware operations and physical success remain unrun. No controller benchmark batch was run.
