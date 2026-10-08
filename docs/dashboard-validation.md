# Local desktop validation

This evidence covers the separate desktop application on the development Mac,
not robot execution. The browser/server fixtures are synthetic. No real
NetworkTables producer, camera calibration qualification, drivetrain physics,
actuator, controller deployment or physical task completion was exercised.

Host: MacBook Pro Mac17,7, Apple M5 Max, macOS26.6.2 AArch64. Java17.0.20.1+1,
Gradle8.5 and Node24.14.0. Java25.0.4.1+1 additionally executed the current common
release-17 core/SPI artifact. Browser checks used actual local in-app Chromium;
no external research page or other workspace was operated. Tests were bounded
CPU work, with one planning worker. The desktop check is not a combined-workload latency benchmark.

## Executed software checks

```sh
./gradlew --offline --no-daemon --max-workers=1 --gradle-user-home ../.gradle-user \
  build worldStateAdapterJar dashboardContractCheck dashboardHttpCheck
node fixtures/field-map/test-field-map.mjs
node fixtures/dashboard/test-scenario-import.mjs
node --check src/dashboard/resources/dashboard/app.js
node --check src/dashboard/resources/dashboard/field-import.js
python3 tools/verify_jdk25.py --java-home "$JDK25_HOME" --release 17
./gradlew --offline --no-daemon --max-workers=1 --gradle-user-home ../.gradle-user \
  runDashboard -PdashboardPort=8086
./gradlew --offline --no-daemon --max-workers=1 --gradle-user-home ../.gradle-user \
  dashboardMockProducer -PmockFrames=30 -PmockIntervalMs=1000 -PdashboardPort=8086
```

Passed: 18 planner acceptance groups, 20 direct owner-envelope/adapter/validator
checks, 30 desktop model checks, 12 loopback HTTP checks, 58 independent JS
field-map checks and eight async scenario-import regressions. The latter execute
the actual handler/decoder/validators in a Node VM and verify newer import/edit and
mode changes retire old reads without error leakage. Syntax checks and strict
Java compilation passed. Java25's
current source/bytecode/jar/dependency report is retained locally as the ignored,
unpublished `docs/evidence/dashboard/jdk25-current.json`; source hashes identify exactly
what was compiled. Earlier 17-group Java25
reports remain historical. Core/adapter jars exclude desktop, Jackson, benchmark,
WPILib and historical demo classes. No warning was suppressed.

HTTP tests used their own loopback port18086 and closed their server. They verified
ordinary state access, foreign Origin/same-site rejection, CLI-only live ingest,
duplicate/trailing JSON rejection, 64KiB body limit, cancel, rebinding Host rejection
and closure of a slow request body before four seconds. These runtime connection
deadlines are qualified on this JDK17 only; JDK25 HTTP behavior is unrun.

## Actual browser observations

| Interaction | Final observed result |
|---|---|
| Plan an editable synthetic Sandbox | Actual Java backend SUCCESS; validated path and measured solver duration; no executable/timed-trajectory claim |
| Start preview, then Cancel | CANCELLED; proposal removed and preview unavailable |
| Edit obstacle radius to1.5m | Value persisted through polling; automatic planning returned NO_PATH |
| Drag start on the observed canvas | Start changed to about(1.495,2.495)m and replanned |
| Replay step/play/pause and accessible timeline scrub | Selected recorded frame3/3; unchanged Sandbox state when returning |
| Run separately authored30-frame CLI mock | Live connected and read-only; no goal/place/plan authority |
| Stop that producer | Source aged stale; path geometry hidden |
| Stop this task's server during preview | DISCONNECTED; preview stopped and start control disabled |
| Generate/import synthetic raster, calibrate and independently check | Early approval rejected until calibration/checks existed |
| Suggest, review boundary/obstacle, approve exact map and save | Revision3 accepted; raw unknown heights preserved; circles/bounds derived and generic editing disabled |
| Edit reviewed outer polygon | Revision4 became draft; save rejected; server's saved revision3 remained unchanged |
| Resize to390px mobile width | Page width390px, canvas364px, no horizontal page overflow |
| Return to Sandbox after Replay/Live | Original edited start and generic field identity retained |

The saved interactive approved content digest was independently recomputed as
`29dc9937a10c4b11be2b524c73bc456521a7fe2059fb765bd2cdfbf6982c6a2d`.
This is a separate generated example from the shared golden fixture. It proves
document consistency, not field accuracy. The unchanged golden fixture's digest
is `e022b3c0b20e83f71f8a6519946d635bc2e730c38d20f65392dae404d76f0163`,
with image SHA256
`9f8ad23da16dcf864327ca31558ad9e1cae8864a8355e9c2f9feba48a828ebeb`.
Custom-Vision independently checked these exact bytes/digests, metric affine
round trips, holes, unknown heights and no added inflation. Its independently reported local
viewer results gave18 scene checks and34 actual Chromium checks; these are separate
consumer evidence, not locally rerun tests of its checkout.

The locally retained, ignored `browser-observations.json` preserves20 chronological
exploratory records, including
intermediate PENDING and an unsuccessful drag using stale screen coordinates. It
is deliberately **not** an all-pass assertion. Final fresh-coordinate drag and
accessible timeline selection succeeded. Earlier actual browser issues prompted
fixes for poll/edit races, replay ordering, HTTP freshness, finite import validation,
preview braking limits, derived geometry editing and stale default field identity;
their affected interactions were repeated after correction. Final review caught
the pending scenario-file import race; its eight regression cases passed, and
the refreshed browser again displayed an actual backend SUCCESS. Browser console
warnings/errors were empty at the final observation. The file chooser loaded a
JSON fixture; paired external image-file picker interaction was not completed.
The generated synthetic raster calibration/review workflow was exercised instead.

## Local deliverables and remaining checks

Screenshot files are retained under ignored `build/dashboard/screenshots/`:
`preview-cover.jpg`, `sandbox-no-path.jpg`, `replay.jpg`, `live-connected.jpg`,
`live-stale.jpg`, `image-approved.jpg`, `http-disconnected.jpg`, `mobile.jpg`,
`sandbox-success.jpg` and `final-preview.jpg`. The locally retained, ignored
`screenshots.json` records exact byte hashes and sizes. Captures and generated
runtime reports are excluded from the public source branch. They depict this
synthetic application only. The foreground
preview binds127.0.0.1:8086 and is stopped with Ctrl-C; no service/autostart was added.

Remaining: genuine owner/NT feed and clock-domain adapter; real field image/metric
measurement and application accuracy thresholds; external image-file chooser
round trip; controller/native deployment; combined robot-stack timing; drivetrain
following/time parameterization and physical verification. WPILib2026+Systemcore
remains unsupported by the recorded official matrix. Robot integration stays on
hold. Source publication performs no package publication, GPU work, hardware
action or large training. The tested source files and fixtures remain unchanged
during publication cleanup; the existing exact-source evidence is reused.
