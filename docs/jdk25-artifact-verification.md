# JDK 25 and alpha-7 artifact verification

Verified 2026-10-08. This check inspected existing files and official release metadata; it ran no builds, contract tests, network loopback, controller deployment or hardware. Compilation and execution evidence is recorded separately in this repository.

## Verified JDK

The cached archive is **Eclipse Temurin 25.0.4.1+1**, HotSpot, macOS AArch64. Its 136,355,078 bytes hash to:

```text
61979887f7506a24a57439ff99adb8b3a7fc89977d9cfe3b8984f58a981b7b9d
```

That SHA-256 matches the [official published checksum](https://github.com/adoptium/temurin25-binaries/releases/download/jdk-25.0.4.1%2B1/OpenJDK25U-jdk_aarch64_mac_hotspot_25.0.4.1_1.tar.gz.sha256.txt) for the [official archive](https://github.com/adoptium/temurin25-binaries/releases/download/jdk-25.0.4.1%2B1/OpenJDK25U-jdk_aarch64_mac_hotspot_25.0.4.1_1.tar.gz), under the [Adoptium release](https://github.com/adoptium/temurin25-binaries/releases/tag/jdk-25.0.4.1%2B1).

The extracted JDK's `release` file identifies Eclipse Adoptium, `JAVA_RUNTIME_VERSION="25.0.4.1+1-LTS"`, `OS_ARCH="aarch64"`, `OS_NAME="Darwin"`. Running that JDK's `java -version` independently returned Temurin **25.0.4.1+1-LTS**, dated **2026-08-18**. This is a Mac host JDK, not a Systemcore operating-system image or native cross toolchain.

Read-only cache location:

```text
<isolated artifact cache>
```

The corresponding lock is `tools/nt4_interop/artifacts.lock.json` in that producer checkout. Neither it nor its result files were changed.

## Verified released alpha-7 artifacts

Every item below is pinned to **2027.0.0-alpha-7**. Independently calculated local SHA-256 and byte counts matched the producer lock and official WPILib Artifactory `api/storage/release` checksum metadata. Links are exact official download endpoints.

| Artifact | Bytes | SHA-256 |
| --- | ---: | --- |
| [ntcore-java-2027.0.0-alpha-7.jar](https://frcmaven.wpi.edu/artifactory/api/download/release/org/wpilib/ntcore/ntcore-java/2027.0.0-alpha-7/ntcore-java-2027.0.0-alpha-7.jar) | 144,352 | `63867e59b758bf63e82fe11b19c6fb6335396c7a828788fcc121ed26dff21e49` |
| [ntcore-cpp-2027.0.0-alpha-7-osxuniversal.zip](https://frcmaven.wpi.edu/artifactory/api/download/release/org/wpilib/ntcore/ntcore-cpp/2027.0.0-alpha-7/ntcore-cpp-2027.0.0-alpha-7-osxuniversal.zip) | 1,109,995 | `49c307870055518bf337fc3518c1905bac9f95bff05b7918a2616f7797f820e5` |
| [wpiutil-java-2027.0.0-alpha-7.jar](https://frcmaven.wpi.edu/artifactory/api/download/release/org/wpilib/wpiutil/wpiutil-java/2027.0.0-alpha-7/wpiutil-java-2027.0.0-alpha-7.jar) | 108,752 | `086803153340e3070b1f0c4a297b4775f920f6ed53cd134c6d8b28ed19d4b664` |
| [wpiutil-cpp-2027.0.0-alpha-7-osxuniversal.zip](https://frcmaven.wpi.edu/artifactory/api/download/release/org/wpilib/wpiutil/wpiutil-cpp/2027.0.0-alpha-7/wpiutil-cpp-2027.0.0-alpha-7-osxuniversal.zip) | 894,560 | `6f786cf70b005389a8c5f171b6c499ec693f19cc6aababc8150c2312fcc9e715` |
| [wpinet-java-2027.0.0-alpha-7.jar](https://frcmaven.wpi.edu/artifactory/api/download/release/org/wpilib/wpinet/wpinet-java/2027.0.0-alpha-7/wpinet-java-2027.0.0-alpha-7.jar) | 7,031 | `68405d182a74cce68dfaff2f9cab08553b09430835cf5ee13eed3b5475fe7494` |
| [wpinet-cpp-2027.0.0-alpha-7-osxuniversal.zip](https://frcmaven.wpi.edu/artifactory/api/download/release/org/wpilib/wpinet/wpinet-cpp/2027.0.0-alpha-7/wpinet-cpp-2027.0.0-alpha-7-osxuniversal.zip) | 1,419,657 | `136ba202fd348b24c091412b56ba39045d676755482c743c0b0c10308819b11b` |
| [datalog-java-2027.0.0-alpha-7.jar](https://frcmaven.wpi.edu/artifactory/api/download/release/org/wpilib/datalog/datalog-java/2027.0.0-alpha-7/datalog-java-2027.0.0-alpha-7.jar) | 41,256 | `1aa1042c101e159824aba21635c9750cce0545c7c06051248365d7584539ae70` |
| [datalog-cpp-2027.0.0-alpha-7-osxuniversal.zip](https://frcmaven.wpi.edu/artifactory/api/download/release/org/wpilib/datalog/datalog-cpp/2027.0.0-alpha-7/datalog-cpp-2027.0.0-alpha-7-osxuniversal.zip) | 344,803 | `281c70de2f9b75263eb51afb97f96ce1e64240332a9d72190b85cbbfdc51a5f7` |

For each link, its verified official checksum metadata URL is obtained by replacing `/api/download/` with `/api/storage/`; for example, [NTCore metadata](https://frcmaven.wpi.edu/artifactory/api/storage/release/org/wpilib/ntcore/ntcore-java/2027.0.0-alpha-7/ntcore-java-2027.0.0-alpha-7.jar). These objects were published on **2026-08-31** according to repository metadata.

All classes in the four Java jars have class-file major version **69**, requiring Java 25. Their inspected package roots are:

| Jar | Classes | Package root |
| --- | ---: | --- |
| NTCore | 130 | `org.wpilib.networktables` |
| WPIUtil | 75 | `org.wpilib.util` |
| WPINet | 7 | `org.wpilib.net` |
| DataLog | 26 | `org.wpilib.datalog` |

The Java jars contain no embedded native libraries. Each `osxuniversal` ZIP contains its library and JNI library under `osx/universal/shared/`. Their Mach-O headers contain both x86_64 (`0x01000007`) and arm64 (`0x0100000c`) slices. These are **Mac desktop native artifacts**, not Systemcore Linux binaries.

## Released coordinates versus source/build profiles

The independently fetched [NTCore POM](https://frcmaven.wpi.edu/artifactory/api/download/release/org/wpilib/ntcore/ntcore-java/2027.0.0-alpha-7/ntcore-java-2027.0.0-alpha-7.pom) declares `org.wpilib.ntcore:ntcore-java:2027.0.0-alpha-7`. The independently fetched [WPIMath POM](https://frcmaven.wpi.edu/artifactory/api/download/release/org/wpilib/wpimath/wpimath-java/2027.0.0-alpha-7/wpimath-java-2027.0.0-alpha-7.pom) declares `org.wpilib.wpimath:wpimath-java:2027.0.0-alpha-7`. Both POMs contain coordinates only and declare no transitive dependencies. WPIMath binaries were not in this inspected cache and were not downloaded or tested.

That POM shape does not mean those libraries are standalone. Pinned [NTCore source build settings](https://github.com/wpilibsuite/allwpilib/blob/v2027.0.0-alpha-7/ntcore/build.gradle) include DataLog and native WPINet/WPIUtil/DataLog dependencies. Pinned [WPIMath source build settings](https://github.com/wpilibsuite/allwpilib/blob/v2027.0.0-alpha-7/wpimath/build.gradle) include Telemetry, Tunables, WPIUnits, Avaje Jsonb, EJML and Quickbuf, among others. A future WPILib boundary must use a complete matching official build profile rather than assume one jar or a namespace substitution supplies its dependencies.

The [alpha-7 README](https://github.com/wpilibsuite/allwpilib/blob/v2027.0.0-alpha-7/README.md) requires JDK 25 and a Systemcore cross toolchain for controller development. Its [wrapper](https://github.com/wpilibsuite/allwpilib/blob/v2027.0.0-alpha-7/gradle/wrapper/gradle-wrapper.properties) pins **Gradle 9.4.1**. This planner's Gradle 8.5 wrapper remains usable with Java 17; it must not be launched under Java 25 or presented as the alpha-7 controller profile. [Gradle's matrix](https://docs.gradle.org/current/userguide/compatibility.html#java_runtime) lists Java 25 toolchain/runtime support from 9.1.0. Direct `javac`/`jar`/`java` verification with the explicit JDK 25 is a separate host compatibility check.

## This backend's actual boundary

Source inspection found **no WPILib or PathPlanner imports** in `src/main/java` or `src/worldStateAdapter/java`. The default core has no external production dependency. `AStarPlannerBackend` depends on the controller-independent pinned World-State API. The historical demo alone selects WPILib **2026.2.2**, Java 17 and `edu.wpi.first.*`; it remains outside the default core and World-State adapter artifacts.

The locally produced artifacts in `build/compatibility/jdk25-release17/` were independently inspected without rebuilding. Every class had major version **61** (Java 17 bytecode, consistent with JDK 25 compilation using `--release 17`). No class-file reference to `org/wpilib/`, `edu/wpi/first/` or `com/pathplanner/` was found.

| Local artifact | Bytes | Observed SHA-256 |
| --- | ---: | --- |
| `geometric-core.jar` | 24,523 | `ec8d9a550d51378b5760400484927c461bf15ab0dc079030835cb44bf3bd9b03` |
| `world-state-adapter.jar` | 19,084 | `e062c5dfd25bd590661c062f49a685085c10a371dfd17b51d527dde59b897c84` |
| `world-state-api.jar` | 37,669 | `b3b8531a2152e0b3d9d40906441c8af70bebbe636cde38fa4bd4154f2c90c06e` |
| `contract-suite.jar` | 13,831 | `2e662735c24b0d8f6f1d13904e6b4d7ea38fb3003d24297cc65277fb9d3b933a` |

These hashes identify the earlier inspected local output snapshot; they are not upstream release pins. The subsequent strict-lint fixes and reproducible repackaging changed the jar hashes. The [final release17 evidence](evidence/jdk25-release17.json) and [release25 evidence](evidence/jdk25-release25.json) identify the tested final outputs. Java 25 `jdeps -s`, using the inspected snapshot, returned:

```text
geometric-core.jar -> java.base
world-state-adapter.jar -> geometric-core.jar
world-state-adapter.jar -> java.base
world-state-adapter.jar -> world-state-api.jar
world-state-api.jar -> java.base
```

This proves the packaged dependency boundary. It does not by itself prove the source build, test execution, NT transport, or controller operation. Producer loopback results are separate from A* test evidence. No controller-specific adapter was invented or implemented.

## API units and remaining controller checks

Alpha-7 [TimestampedString](https://github.com/wpilibsuite/allwpilib/blob/v2027.0.0-alpha-7/ntcore/src/generated/main/java/org/wpilib/networktables/TimestampedString.java) documents local `timestamp` and `serverTime` in **nanoseconds**; [NetworkTableInstance](https://github.com/wpilibsuite/allwpilib/blob/v2027.0.0-alpha-7/ntcore/src/generated/main/java/org/wpilib/networktables/NetworkTableInstance.java) documents `getServerTimeOffset()` in nanoseconds. JSON `_us` fields and this API's robot timestamps remain **microseconds**. The A* core consumes no NT metadata and measures only monotonic solver durations in nanoseconds.

Primary immediate target remains **WPILib 2026.2.2 + roboRIO**. The separate documented Systemcore target is **WPILib 2027.0.0-alpha-7**, Java 25, with Systemcore image **≥14**. **WPILib 2026 + Systemcore has no official supported matrix entry.** [Official Systemcore testing matrix](https://github.com/wpilibsuite/SystemcoreTesting#software-compatibility).

Systemcore/roboRIO deployment, native loading on either controller, timing/performance qualification, physical path following and actual robot integration remain unrun. Robot integration remains on hold. No hardware or GPU was used for this verification.

## Executed verification

Read-only Python SHA-256/ZIP/class-header/Mach-O inspection; reads of the JDK `release` file and producer lock; cached `java -version`; source import search; own output jar inspection; Java 25 `jdeps -s`; official release/source browsing; and approved read-only HTTPS requests for the official JDK checksum, two Maven POMs and eight artifact checksum metadata records. Initial browser-only POM/checksum requests were unavailable; the supported read-only HTTPS requests succeeded. An initial `jdeps --summary` spelling was rejected; the corrected `jdeps -s` invocation succeeded. This independent artifact inspection did not execute builds or tests.
