The telemetry/replay files are tiny authored SYNTHETIC display fixtures, not
robot logs or solver measurements. `python3 fixtures/dashboard/generate-fixtures.py` reproduces those two files
without external packages, planning, network calls or hardware.

`synthetic-telemetry.json` is accepted by the CLI-only live ingest endpoint when
its session/sequence is fresh. Reposting the same sequence is deliberately
rejected. `synthetic-replay.json` is a deterministic three-frame replay load.
Long metadata values use lossless decimal strings. Solver duration0 identifies
authored geometry; it is not a benchmark value. Missing obstacles remain empty.

For continuing live demonstration use `pathplanning.dashboard.SyntheticMockProducer`
instead of replaying the same fixture sequence. It posts at most50 frames to
127.0.0.1 and stops; the live view then becomes stale after one second.

Run the opt-in scenario-file import regression from the repository root:

```sh
node fixtures/dashboard/test-scenario-import.mjs
```

It uses Node's built-in modules and executes the actual `app.js` importer and
validators with controlled asynchronous file reads. Eight checks cover a valid
typed import, switching to Live/Replay, leaving and returning to Sandbox, newer
edits/imports, and current versus obsolete invalid JSON. The test does not start
a browser, server, planner, benchmark, or hardware connection.
# Dense curve course

`curve-gauntlet.json` is a deterministic 12×6 m synthetic Sandbox scenario with
26 raw obstacle circles, a 0.6×0.6 m robot and a route requiring alternating
detours. Load it through **Import scenario**, then Plan. Its supplied geometry is
not an official game field or a physical measurement. The same course is covered
by `CurvedMotionIntegrationSuite` so tiny corner fillets cannot satisfy the new
whole-route geometry regression. [Results and limits](../../docs/curved-motion.md)
describe actual curve certification, continuous curvature and timed motion.
