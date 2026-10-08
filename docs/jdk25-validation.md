# Mac Java 25 compatibility and owner integration evidence

Follow-up completed 2026-10-08 in this A* checkout. Production remains a controller-independent geometric core plus its `frc-planner/1` World-State SPI implementation. **There is no WPILib-specific 2027 runtime adapter in this backend, and none is required by its dependency boundary.** The alpha-7 controller pin is separate metadata; these results qualify only pure-Java compilation, packaging and CPU execution on this Mac.

## Executed profiles

| Compiler / profile | Packaged bytecode | Acceptance groups | Owner-envelope integration checks | Smoke / dependencies |
| --- | --- | ---: | ---: | --- |
| Temurin17.0.20.1, strict `--release17 -Xlint:all -Werror` | Java17 classes; default Gradle jars | 17 passed | 20 passed | CPU synthetic; no controller libraries |
| Temurin25.0.4.1+1, strict `--release17 -Xlint:all -Werror` | Major61, separately packaged jars | 17 passed | 20 passed | Smoke passed; `jdeps` only `java.base`, core, owner API |
| Temurin25.0.4.1+1, strict `--release25 -Xlint:all -Werror` | Major69, separately packaged jars | 17 passed | 20 passed | Smoke passed; identical dependency boundary |

The Java25 script recompiles all sources locally, packages API/core/adapter/tests separately with fixed jar timestamps, and runs tests from those jars. It uses freshly compiled sources and does not load native libraries. All outputs are generated here under `build/compatibility/`. The explicit JDK executable is read-only from the producer's official cache; its archive SHA256 was independently checked against Adoptium's published checksum. [Official artifact verification](jdk25-artifact-verification.md) records exact alpha-7/JDK URLs, sizes, package roots and checksums.

Committed evidence: [release17](evidence/jdk25-release17.json), [release25](evidence/jdk25-release25.json). Each contains command arguments, compiler/runtime versions, jar SHA256, class-file release, complete suite/owner-integration/smoke outputs, and `jdeps` results. Private absolute paths are normalized to `$CHECKOUT` and `$JDK25_HOME`; artifact hashes and test results are unchanged. Future verification runs apply the same normalization. The release17 common-bytecode profile remains the recommended production artifact; release25 binaries cannot run on the Java17 roboRIO target.

```sh
python3 tools/verify_jdk25.py --java-home /absolute/path/to/JDK25 --release 17
python3 tools/verify_jdk25.py --java-home /absolute/path/to/JDK25 --release 25
```

The tested JDK25 path was `$JDK25_HOME`. No global install or another checkout modification was performed. The existing Gradle8.5 wrapper remains on JDK17; no Gradle8.5-on-Java25 compatibility is claimed. An eventual WPILib alpha-7 controller project has its own Gradle9.4.1/JDK25/Systemcore toolchain.

## Direct owner-computed envelope test

The integration test consumes exact owner implementations `ObstacleEnvelopeBuilder` and `PlannerValidation`, from World-State commit **`1f8b9aace3fd320e3545744e3125aa348089bc99`**, compiled here as test-only fixtures against the held API. [Fixture provenance](../vendor/world-state-test-fixtures-src/PROVENANCE.md) records hashes. Neither owner helper is included in production jars.

A synthetic coasting track has physical radius0.3m, bounded motion1m/s, last measurement age0.2s, future request horizon0.2s, timing error0.02s, additional latency0.03s and two-sigma positional uncertainty0.2m. The owner builds a radius0.95m envelope anchored at the reconstructed measured center: `0.3 + 0.2 + 1 * (0.2 + 0.2 + 0.02 + 0.03)`. A* accepts that envelope, adds raw robot footprint/clearance once, and returns a detour with preserved IDs and null timed trajectory. The owner's independently implemented circle/segment validator accepts the complete returned geometry.

The same test rejects stale measurements, unconfigured geometry, empty tracks without independently fresh coverage, changed map/epoch/field and unqualified timed data. It also verifies static geometry and exact age/uncertainty/motion/timing/latency expansion.

Required upstream contract: each obstacle envelope must already cover **physical dimensions, observation age/coasting, uncertainty and all bounded swept motion through request expiry**, including relevant timing/latency bounds. Envelopes exclude robot footprint inflation. v1's omitted separate dimensions/age fields cannot be reconstructed by A*. Empty detections do not establish free space; stale, missing or unbounded geometry must reject upstream output. The pinned owner builder supplies this envelope contract.

## Fixes and remaining limits

World-State's strict compile found three serialization lint warnings in internal exception classes. Added `serialVersionUID` to `GeometricSolver.Stopped`, `AStarPlannerBackend.Cancelled` and `Expired`; strict JDK17/JDK25 compilations now pass. During active owner-source export, a helper hash changed; the pin check stopped before compilation. The copied source was rechecked against the owner, repinned to its committed hash and tested. PlannerBackend schema/API pin did not change.

No WPILib jars are present in either test runtime classpath, and production class-file inspection rejects WPILib/PathPlanner references or bundled owner/season classes. This proves this backend's empty WPILib dependency profile, rather than testing an invented 2027 adapter. Official alpha-7 artifact inspection is separate provenance evidence; it supplies no NT/native/controller execution claim.

Controller deployment/native loading, timed following, actual robot integration, target latency/memory qualification and physical success remain unrun/on hold. **WPILib2026+Systemcore remains unsupported in the official matrix.** Java25 Mac execution does not change that status.
