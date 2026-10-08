# Controller/toolchain compatibility verification

Verified 2026-10-08 from official WPILib sources. This report concerns software targets; no controller deployment, robot execution, or hardware qualification was performed.

## Target decision

| Requested target | Exact software pin | Java/API boundary | Evidence and status |
| --- | --- | --- | --- |
| Primary: WPILib 2026 + roboRIO | `v2026.2.2` | JDK 17; `edu.wpi.first.*` | The pinned allwpilib README identifies roboRIO and requires JDK 17. [Pinned README](https://github.com/wpilibsuite/allwpilib/blob/v2026.2.2/README.md), [tag](https://github.com/wpilibsuite/allwpilib/releases/tag/v2026.2.2). |
| Separate: WPILib 2027 + Systemcore | `v2027.0.0-alpha-7` | JDK 25; `org.wpilib.*` | Documented alpha testing target. The pinned README identifies Systemcore and requires JDK 25. [Pinned README](https://github.com/wpilibsuite/allwpilib/blob/v2027.0.0-alpha-7/README.md), [alpha release](https://github.com/wpilibsuite/allwpilib/releases/tag/v2027.0.0-alpha-7). |
| Requested: WPILib 2026 + Systemcore | **No official supported pin found** | Unsupported combination | The current official matrix lists only 2027 alpha toolchains. Do not advertise a 2026 Systemcore build, relabel a 2027 build as 2026, or invent a HAL port. [Official matrix](https://github.com/wpilibsuite/SystemcoreTesting#software-compatibility). |

The official Systemcore image/WPILib matrix currently maps images ≤9 to alpha-1/2, images 10–13 to alpha-5/6, and images ≥14 to alpha-7 or later. It also states that its alpha software is incompatible with roboRIO and Control Hub. Alpha-7 testing therefore needs an appropriate Systemcore image, with the exact hardware revision and image recorded before qualification. No image has been selected or flashed here. Offseason event FMS compatibility also requires a later independent check. [SystemcoreTesting](https://github.com/wpilibsuite/SystemcoreTesting#software-compatibility).

## Recommended build boundaries

These are implementation recommendations, not completed controller validation:

- Keep the geometric core and World-State API dependency free of WPILib and compile them with `--release 17`. Java 17 bytecode keeps one common artifact usable by the 2026 Java 17 and 2027 Java 25 runtimes. Keep immutable SI-valued geometry, snapshot IDs, epochs, and monotonic elapsed durations in that core.
- Put all `edu.wpi.first.*` conversions in a 2026 adapter/profile pinned to WPILib `2026.2.2`. Put all `org.wpilib.*` conversions in a separate Java 25 alpha-7 adapter/profile pinned to `2027.0.0-alpha-7`. Never include both adapters in one runtime classpath.
- The pinned allwpilib source wrappers are Gradle **8.14.3** for 2026.2.2 and **9.4.1** for alpha-7. These are evidence about upstream source builds, not proof that this planner's historical wrapper is controller ready. If adding full controller build profiles later, use the matching official generated project/toolchain and test each profile independently. [2026 wrapper](https://github.com/wpilibsuite/allwpilib/blob/v2026.2.2/gradle/wrapper/gradle-wrapper.properties), [alpha-7 wrapper](https://github.com/wpilibsuite/allwpilib/blob/v2027.0.0-alpha-7/gradle/wrapper/gradle-wrapper.properties).
- Gradle 8.5 can remain for the pure Java 17 core. It is not a supported Java 25 build runtime/toolchain: Gradle lists Java 25 support from 9.1.0. Use the separately pinned Gradle 9.4.1 wrapper for an alpha-7 adapter build. [Gradle compatibility matrix](https://docs.gradle.org/current/userguide/compatibility.html#java_runtime).
- No PathPlanner dependency is selected for this geometric component. Motion time parameterization and following remain external responsibilities. Java 21/WPILib 2024 binaries are not a qualified Systemcore artifact.

## Time-unit migration

The pinned 2026 `NetworkTableInstance.getServerTimeOffset()` returns microseconds; the pinned alpha-7 method returns nanoseconds. Alpha-7 `TimestampedString.timestamp` and `serverTime` explicitly use nanoseconds. Version-specific adapters must normalize metadata with explicit units and time-base identity. JSON fields ending `_us` remain microseconds by the project contract; they do not change when the NT API changes. Search elapsed duration should use monotonic `System.nanoTime()` differences and must never be interpreted as robot trajectory duration. [2026 offset source](https://github.com/wpilibsuite/allwpilib/blob/v2026.2.2/ntcore/src/generated/main/java/edu/wpi/first/networktables/NetworkTableInstance.java), [alpha-7 offset source](https://github.com/wpilibsuite/allwpilib/blob/v2027.0.0-alpha-7/ntcore/src/generated/main/java/org/wpilib/networktables/NetworkTableInstance.java), [alpha-7 value source](https://github.com/wpilibsuite/allwpilib/blob/v2027.0.0-alpha-7/ntcore/src/generated/main/java/org/wpilib/networktables/TimestampedString.java).

## Local evidence and unrun checks

Read-only inspection of `<A* checkout>` found HEAD `e6e8c73821330a5edb5da02658da2fe32ec8da45` and a clean working tree at inspection. `build.gradle` still selected Java 21 and WPILib `2024.3.2`; its wrapper selected Gradle 8.5.

Executed: `git rev-parse HEAD`, `git status --short`, `rg --files --hidden` for instructions, reads of `build.gradle` and wrapper properties, `/usr/libexec/java_home -V`, `java -version`, and `command -v gradle`. Registered/default Java was Temurin **17.0.20.1+1**, arm64; no standalone `gradle` executable was found. The initial artifact inspection did not run a build. Optional `git ls-remote` checks failed because the sandbox could not resolve `github.com`; official pages and pinned source files were successfully read through the web tool.

Still unrun: 2026 adapter compile/desktop checks; Java 25 alpha-7 adapter compile/desktop checks; Systemcore image qualification; actual roboRIO/Systemcore timing, native-library loading and motion tests. Those require their own evidence and do not follow from source inspection. Actual robot integration remains on hold.

## Follow-up: completed pure Java 25 host checks

The earlier unrun statements describe the initial source-inspection milestone.
The production geometric core and World-State SPI adapter have since been
compiled, packaged and executed directly with the verified macOS AArch64
Temurin **25.0.4.1+1** JDK. The completed `--release 17` record at
`build/compatibility/jdk25-release17/result.json` reports **17 acceptance groups**,
**20 direct World-State computed-envelope/adapter/owner-validator checks**, and
the synthetic smoke example passing. Its class-file major version is 61; its
`jdeps` output lists only `java.base`, the geometric core and the pinned owner API.
This is Mac JVM evidence. Hardware and timed following are explicitly false in
that record. Final per-release results, including the separate `--release 25`
checks, are recorded in [JDK 25 validation](jdk25-validation.md).

The production backend consumes no WPILib or NT API, so a distinct WPILib 2027
adapter/native controller profile is not required by this component. No such
adapter was implemented. Any future robot project that imports `org.wpilib` or
loads controller-native libraries still needs its matching official profile.
The historical WPILib 2026 desktop demo remains outside production artifacts;
its later verification is tracked in [validation](validation.md).

Reproduce host verification with `python3 tools/verify_jdk25.py --java-home
<explicit-isolated-cached-JDK25-Contents/Home> --release 17` or `--release 25`.
Portable JDK override commands are in the [README](../README.md). The script
uses direct `javac`/`jar`/`java` tools; it does not launch Gradle or change global
JDK configuration. **Gradle 8.5 must remain on JDK 17.** Verified JDK and released
alpha-7 artifact provenance are in
[artifact verification](jdk25-artifact-verification.md).

Controller/native loading, target timing, physical following and actual robot
integration remain unrun/on hold. The immediate target stays WPILib 2026.2.2 +
roboRIO. The separate Systemcore pin stays 2027.0.0-alpha-7/Java 25/image ≥14;
WPILib 2026 + Systemcore still has no official supported matrix entry.
