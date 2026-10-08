# Historical demo migration

The audited baseline `e6e8c73821330a5edb5da02658da2fe32ec8da45` used Java 21,
WPILib 2024.3.2, and the season-specific `OnTheFlyPlannerMain` as the default
application. That main method marked an autonomous objective complete whenever
search returned a path. The legacy `Trajectory` reconstruction stored
`planning.timeout_ms` as its nominal motion duration. Those findings were
rechecked against this checkout before modification.

## Artifact boundary

The original `pathplanning.config`, `control`, `examples`, `field`, `hybrid`,
`integration`, `robot`, and `util` packages now live under
`src/historicalDemo/java`, with their package names preserved. They are excluded
from the production jar, application distribution, and normal runtime classpath.
The opt-in historical source set has an exact WPILib 2026.2.2 pin and uses Java 17.
It retains the old algae, processor, and barge target selection as a synthetic
comparison; these types are not the current planner's domain model.

The default application is `pathplanning.geometric.PlannerSmokeExample`.
The production core has no WPILib, PathPlanner, Jackson, or controller dependency.
The optional World-State implementation lives in `src/worldStateAdapter/java`;
the dependency direction is backend to the exact exported World-State API.
`vendor/world-state-api-src` is the pinned API source, not a new contract.
Its local jar is built separately; it is not bundled inside the adapter jar.

Tracked generated `build/` and `.gradle/` artifacts were removed and are ignored.
No repository visibility, remote, or deployment setting was changed.

## Historical API change

`pathplanning.util.Trajectory` is now `pathplanning.util.GeometricPath`.
`getNominalDuration()` is replaced by `getPlanningDurationMeasured()`.
The returned `Duration` measures solver elapsed time with `System.nanoTime()`;
it is independent of the configured search budget and supplies no robot velocity,
acceleration, feedforward, or time parameterization.

The synthetic integration main no longer calls `markObjectiveComplete()` after
a successful search. The historical coordinator and explicit completion method
remain available only in that opt-in source set for comparison. An external
execution owner must supply physical completion evidence. The production backend
has no match-mode transition or objective-completion API.

Historical `FieldMap` now rejects non-finite and out-of-bounds coordinates before
grid indexing; it no longer clamps them into valid cells. Maps must have positive
finite dimensions and resolution, a rectangular nonempty grid, and cells that
cover their declared dimensions. Occupancy queries conservatively reject invalid
coordinates or margins. The historical hybrid solver rejects invalid or occupied
start and goal poses, and uses monotonic elapsed time for timeout checks.

These corrections do not qualify the historical solver for execution. Its sampled
collision check and mutable season objective handling retain the baseline's
limitations. In particular, its target list clears on an empty detection update;
that comparison code must not be reused as production obstacle tracking.

## Build commands

Use the supplied Gradle 8.5 wrapper and JDK 17 for the controller-independent core:

```sh
./gradlew --no-daemon clean check jar worldStateAdapterJar
./gradlew --no-daemon run
```

`check` runs the CPU-bounded `pathplanning.geometric.ContractSuite` through
`contractCheck`. The exact pinned World-State API sources compile into
`build/libs/frc-world-state-api-1.jar`. To test against an independently built
copy of that same pin, supply `-PworldStateApiJar=/absolute/path/to/api.jar` and
`-PworldStateApiSha256=<recorded-SHA256>`.
Verify its provenance and checksum before overriding; a different API jar is a
contract migration and requires adapter review.

The historical source set is opt-in:

```sh
./gradlew --no-daemon historicalDemoClasses
./gradlew --no-daemon runHistoricalDemo
./gradlew --no-daemon runHistoricalDemo -PhistoricalMainClass=pathplanning.integration.OnTheFlyPlannerMain
```

These commands use local synthetic poses; they do not connect to or operate a
robot. The last command runs the old season-loop comparison for about 17 seconds.
Neither historical entry point is part of the normal application distribution.

## Separate controller pins

`targets/wpilib-2026-roborio.properties` records the immediate official target:
WPILib 2026.2.2, Java 17, roboRIO, and `edu.wpi.first` APIs.

`targets/wpilib-2027-alpha7-systemcore.properties` records a separate target:
WPILib 2027.0.0-alpha-7, Java 25, Systemcore image 14 or newer, `org.wpilib` APIs,
and the upstream Gradle 9.4.1 pin. These files are target metadata; they do not
select an implemented controller adapter. Gradle 8.5 cannot be used as a supported
Java 25 build toolchain. The production core and World-State SPI adapter use no
WPILib or native APIs and require no distinct 2027 adapter. Their Java 17 bytecode
has now been compiled, packaged and tested with Temurin 25.0.4.1 on the Mac;
17 acceptance groups and 20 direct World-State computed-envelope/owner-validator
checks passed. The isolated historical demo still uses `edu.wpi.first`/WPILib
2026.2.2 and has no Systemcore compatibility claim.

Reproduce the pure Java host checks with the explicit cached JDK; the script
does not change Gradle's runtime or install a JDK globally:

```sh
python3 tools/verify_jdk25.py --java-home "$JDK25_HOME" --release 17
python3 tools/verify_jdk25.py --java-home "$JDK25_HOME" --release 25
```

`--release 17` retains the shared Java 17 artifact; `--release 25` checks a separate
Java 25 bytecode build. The verified release-17 record is
`build/compatibility/jdk25-release17/result.json`. See
[artifact provenance](jdk25-artifact-verification.md) and
[final per-release evidence](jdk25-validation.md). Keep this Gradle 8.5 wrapper
on JDK 17. A future controller project containing `org.wpilib` or native calls
needs its own official Gradle 9.4.1/JDK 25 profile. Controller loading, timing,
path following and robot-code integration remain unrun and on hold.

The official Systemcore testing matrix checked on 2026-10-08 lists 2027 alpha
toolchains only; it does not list an official WPILib 2026 + Systemcore target.
`targets/wpilib-2026-systemcore.properties` records that unsupported request.
No HAL compatibility layer, namespace rewriting, or invented controller support
is supplied. The offseason game year does not choose the controller toolchain.
See [target verification](target-verification.md) for primary references.

## Verification scope

Implemented: artifact isolation, exact historical WPILib pin, measured solver
duration, empty-frame limitations documented, fail-closed historical grid indexing,
and removal of search-triggered objective completion.

Unit-tested/simulated: see the production contract-suite output and README for the
current core results. Historical compilation and examples are reported separately
when run; their success does not establish controller or drivetrain compatibility.
The Mac Java 25 pure Java release-17 acceptance and owner-integration checks now
pass; they establish host JVM compatibility of this backend's packaged boundary.

Physically verified: none. Robot execution, time parameterization/following,
execution guards, PathPlanner conversion, drivetrain units/module order, and
Systemcore adapter testing remain separate integration work on hold.
