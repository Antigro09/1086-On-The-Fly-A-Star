# 1086 geometric A* backend

This library proposes collision-validated **geometric paths** for an immutable World-State request. World-State and robot code choose tasks, maintain targets and match state, guard execution, and verify physical success. A successful search completes no physical objective. Robot-code integration is on hold.

The production core is Java 17 with no WPILib, NetworkTables, PathPlanner or native dependency. Its adapter implements World-State `frc-planner/1`; its `PlannerBackend` source matches [public World-State revision `0e5b6c3`](https://github.com/Antigro09/FRC-World-State/tree/0e5b6c3f85d47205cde8b0c80e13fa79d35e95ab). The standalone build retains earlier support copies with exact hashes and differences documented in [API provenance](vendor/world-state-api-src/PROVENANCE.md). The season-specific algae/barge demonstration is preserved in a separate historical source set and excluded from production jars.

## Build and run

Use JDK 17 and the pinned Gradle 8.5 wrapper. Downloads are needed on the first wrapper run. No Jetson, controller or GPU is needed.

```sh
./gradlew --no-daemon --max-workers=1 build worldStateAdapterJar
./gradlew --no-daemon --max-workers=1 run
./gradlew --no-daemon --max-workers=1 contractCheck
```

`run` executes `pathplanning.geometric.PlannerSmokeExample`, a short synthetic obstacle example. `build/libs/on-the-fly-a-star-0.2.0.jar` contains the core; `on-the-fly-a-star-0.2.0-world-state-adapter.jar` adds the adapter. The adapter jar does not bundle World-State. The pinned source produces a separate `frc-world-state-api-1.jar` for local tests; actual integration must use World-State's pinned API artifact.

The dependency-free acceptance suite can also be compiled using `javac --release 17`; the exact command is in [validation.md](docs/validation.md). [API provenance](vendor/world-state-api-src/PROVENANCE.md) records the source pin and hashes. [Migration notes](docs/api-migration.md) describe removed contracts and ownership.

The default build uses the pinned source copies under `vendor/` and requires no sibling checkout. No World-State Maven artifact is published or assumed. To use an API jar built from an explicitly supplied compatible World-State checkout, pass both its path and SHA256:

```sh
./gradlew --no-daemon --max-workers=1 build worldStateAdapterJar \
  -PworldStateApiJar=/absolute/path/to/your/frc-world-state-api.jar \
  -PworldStateApiSha256=THE_VERIFIED_JAR_SHA256
```

This override is deliberate and hash-checked; it does not locate or build another checkout automatically. Record the jar's source revision and retain the `frc-planner/1` API/source pin. API upgrades require coordinated fixture updates and acceptance checks.

Direct Java 25 host verification uses your explicit JDK installation without changing the Gradle runtime. Set `JDK25_HOME` to the directory containing its `bin/java` and `bin/javac`. The recorded verification used the official Temurin 25.0.4.1+1 macOS AArch64 archive linked in the provenance report:

```sh
export JDK25_HOME=/absolute/path/to/your/jdk-25/Contents/Home
python3 tools/verify_jdk25.py --java-home "$JDK25_HOME" --release 17
python3 tools/verify_jdk25.py --java-home "$JDK25_HOME" --release 25
```

The script compiles from clean output directories, packages, checks bytecode/dependencies, and runs CPU acceptance and owner-integration suites on the Mac. `--release 17` preserves the common Java 17 artifact; `--release 25` checks a separate Java 25 bytecode build. Both profiles passed all 17 acceptance groups and 20 checks using World-State's computed envelopes and owner validator on Temurin 25.0.4.1. See [JDK/artifact provenance](docs/jdk25-artifact-verification.md) and [final per-release validation](docs/jdk25-validation.md). Keep the Gradle 8.5 wrapper on JDK 17; these direct compiler/runtime checks do not qualify Gradle 8.5 for Java 25.

## Entry points and ownership

- `AStarPlannerBackend(LongSupplier robotTimeUs)` implements `PlannerBackend`. Supply the **robot monotonic microsecond domain** used by the request; never substitute NT server timestamps or `System.nanoTime()` values. Its synchronous `plan` must run off the robot periodic thread.
- `LatestRequestPlanner` supplies a single daemon worker and one replaceable pending request. It starts disabled. Call `setEnabled(true)`, `invalidate(authoritativeSnapshot)` and `submit(request)` at the appropriate external boundaries. Read geometry through `currentProposal(robotNowUs)`. Futures are diagnostic; callbacks must use async continuations.
- Disable, cancel, task replacement, pose reset, source restart and snapshot/map/field changes must call the gate's disable/cancel/invalidate methods immediately. This removes proposals and cancels active work. World-State's robot execution guard remains required on every cycle.

[Async gate details](docs/async-gate.md) explain publication and context rules. Every result echoes request/task IDs, epoch, snapshot ID and obstacle-map version; field identity remains in its pinned request and gate context. `solverDurationNanos` measures computation, including conversion/validation. `trajectory` is always null. No solver timeout, waypoint count or heading difference is a motion duration.

Use one planning-worker owner: World-State's worker can call the synchronous backend directly, or a standalone caller can use this optional gate. Keep the original immutable request snapshot while that work is pending. A display/periodic publication counter alone is not a new planning snapshot; invalidate when the authoritative planning context or execution guard changes. The gate deliberately rejects a changed pinned snapshot ID and never decides which world changes are meaningful.

## Geometry and collision contract

The solver uses eight-connected holonomic A* with bounded cells, queue entries, expansions and elapsed monotonic time. It connects exact start/goal poses and applies validated line-of-sight simplification. Translation preserves holonomic heading; a separate final stationary rotation reaches the requested heading. It provides no drivetrain feasibility, feedforward, velocity profile or curved trajectory claim.

The robot's raw length and width become a circumscribed disk plus configured clearance. **This backend alone adds robot footprint inflation.** Static grid cells and World-State obstacle envelopes are raw occupancy and must not already include robot radius. Capsule-versus-rectangle checks cover every point along segments, diagonal corners, all footprint rotations, field edges, and the entire position-tolerance region. Reconstruction, simplification and API conversion each receive another collision check. Touching occupancy is collision. The disk and circle-to-box conversion deliberately reject some physically possible narrow passages; grid discretization also limits completeness.

World-State supplies static and dynamic obstacle circles that conservatively cover physical object dimensions, uncertainty, observation age, and the complete request validity interval. The adapter converts each circle to a containing axis-aligned box. API v1 exposes radius/uncertainty/validity, but no separate dimensions or observation-age field; the adapter records age as **unknown**, and trusts the documented enclosing-envelope contract. World-State must produce that envelope and retain/coast observations across empty frames. This backend has no frame-ingestion or obstacle-erasure method; each request takes an immutable snapshot. It checks each supplied envelope's validity and rejects obsolete publication through the gate. An omitted obstacle cannot be inferred here.

These envelopes are conservative swept occupancy, **not space-time planning**. The solver assumes an obstacle may occupy any point in its envelope throughout the request interval. It does not align a robot timed trajectory with forecast time, choose when to cross a moving obstacle, or claim that PathPlanner bounding boxes contain velocity/covariance. Invalid inputs, unsafe endpoints, resource exhaustion, timeout or cancellation return typed failures with no path. A separate robot guard must stop execution when sensing or authority changes regardless of replanning.

## Controller targets

| Target | Exact pin | Status |
| --- | --- | --- |
| Immediate offseason target: roboRIO | WPILib 2026.2.2, JDK 17 | Common core and API adapter tested on Mac; optional historical WPILib desktop source set separately checked. Robot integration/hardware unrun. |
| Separate Systemcore target | WPILib 2027.0.0-alpha-7, JDK 25, image ≥14 | Pure Java core/SPI adapter compiled, packaged and tested on Mac JDK 25 with release-17 bytecode. Controller/native loading, timing and robot integration unrun. |
| WPILib 2026 + Systemcore | No official supported target found | Unsupported by the current official testing matrix. |

See [official-source verification](docs/target-verification.md) and `targets/`. Offseason game year does not choose the controller toolchain. Java 21/WPILib 2024 binaries from the audited baseline are removed. Alpha-7 moves to `org.wpilib.*`; its NT metadata is nanoseconds, while 2026 metadata and JSON `_us` remain microseconds. This component uses neither NT API; any external transport boundary must normalize units separately. The production core and World-State SPI adapter need no native WPILib/NT profile or distinct 2027 adapter. Their packaged dependencies are only `java.base`, the core, and World-State's owner API. The historical WPILib 2026 demo remains isolated. Gradle 8.5 serves the Java 17 build; a future native alpha-7 controller project must use its separately pinned Gradle 9.4.1/JDK 25 profile.

PathPlanner is not selected. A future integration must pin its exact conversion/custom-Pathfinder API, assign one owner for time parameterization/following, supply current velocity/rotation and verified drivetrain constraints, and collision-check the timed result. Its endpoint replanning limitation cannot supply a stop mechanism. Robot integration owns feedforward units and module order.

## Capability evidence

| Capability | Implemented | Unit/contract tested | Simulated | Physically verified |
| --- | --- | --- | --- | --- |
| Geometric holonomic search, static/dynamic envelopes, swept footprint, simplification | Yes | Yes | CPU synthetic suite/example | No |
| Immutable snapshot, invalid-input checks, timeout/resource/cancellation bounds | Yes | Yes | CPU synthetic suite | No |
| Latest-request publication, disable/cancel/reset/map/source invalidation | Yes | Yes, including uncooperative old solver | Threaded synthetic suite | No |
| Motion timing, drivetrain following, physical task completion, controller integration | No | No | No | No |
| Pure Java core/World-State SPI adapter on Mac Java 25 | Yes; common release-17 artifact | 17 acceptance groups + 20 owner-integration checks passed | CPU synthetic suite/example | No |
| Systemcore controller/native qualification and WPILib 2026 + Systemcore | Alpha-7 target pinned; 2026 combination unsupported | Controller checks unrun | Unrun | No |

Executed commands, tool versions, results and remaining checks are in [validation.md](docs/validation.md). There is no measured robot planning-frequency claim. [benchmark-procedure.md](docs/benchmark-procedure.md) gives an **unrun** target benchmark procedure with latency distributions, deadline misses, load and memory. No hardware operation, deployment, package publication or push is part of this component work.
