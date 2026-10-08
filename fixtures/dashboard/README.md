These are tiny authored SYNTHETIC display fixtures, not robot logs or solver
measurements. `python3 fixtures/dashboard/generate-fixtures.py` reproduces them
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
