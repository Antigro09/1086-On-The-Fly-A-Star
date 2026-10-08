`pathplanning.dashboard.DashboardServer` is the desktop app entry point. Run the
`runDashboard` Gradle task, optionally `-PdashboardPort=8086`; direct
Java launch accepts `--port 8086`. It binds **127.0.0.1 only**. The desktop source
set depends on Jackson 2.17.2 and the pinned planner API; it is excluded from the
controller-independent core/adapter jars.

The shipped generic 8×4 m sandbox, replay and CLI producer are conspicuously
`synthetic_mock`. No real telemetry feed, NetworkTables connection, actuator,
robot following, physical task completion or hardware qualification is implemented.
Offline planning calls the exact `AStarPlannerBackend` through `LatestRequestPlanner`
with one worker and one replaceable pending request. Replay/live ingest never call
the planner and never change sandbox goals or obstacle maps.

HTTP routes are pinned:

| Route | Body / query | Response |
|---|---|---|
| `GET /api/state` | `mode=sandbox\|replay\|live`, optional replay `index` | normalized state |
| `POST /api/sandbox/scenario` | full/partial scenario, or `{scenario:{...}}` | updated state; old proposal revoked |
| `POST /api/sandbox/plan` | `{budget_ms:250,valid_for_ms:30000,task_id:"sandbox-task"}` | HTTP 202 state; poll for completion |
| `POST /api/sandbox/cancel` | `{}` | state with no current proposal |
| `POST /api/replay/load` | synthetic replay document below | recorded frame zero |
| `GET /api/replay/sample` | `index=0` | exact recorded frame and replay metadata |
| `POST /api/live/ingest` | normalized synthetic telemetry frame | read-only live state; CLI producer only |

`/`, `/index.html`, `/app.js`, `/style.css` and `/field-import.js` serve only named
classpath resources under `dashboard/`. No filesystem path, arbitrary URL,
upload-to-cloud or operating-system endpoint is exposed. All mutations require
`application/json`. Ordinary bodies are bounded at 64 KiB; replay load at 256 KiB.
Duplicate JSON keys/trailing values are rejected, nesting is at most 32, and parser
string/number lengths are bounded. Responses are bounded at 512 KiB. Live/replay
paths have at most 512 vertices and circles at most 128; excess input is rejected.
Sandbox solver output exceeding 512 vertices remains an explicitly undisplayed
path (`display_limit_exceeded:true`, empty `positions`, original `point_count`);
no validated vertex is dropped, straightened or silently simplified for transport.

The server validates loopback peer and exact `Host:127.0.0.1:<port>` or
`Host:localhost:<port>` on every request. Browser mutations additionally require
an exact same-server `Origin` when supplied and `Sec-Fetch-Site:same-origin|none`.
Live ingest rejects **any** Origin or Sec-Fetch header; the browser only reads live
state. No CORS allowance is emitted. HTTP executor bounds are two workers, queue
16 and listen backlog16. Selected JDK17 HTTP-server properties additionally cap
connections32, idle connections8, headers64/8 KiB, request elapsed time2 seconds,
response elapsed time5 seconds and idle time10 seconds. Timer checks occur every
200 ms; these process-local app settings do not change OS configuration. Their
runtime deadline behavior must be tested on a changed JDK before claiming support.

Every state uses `schema_version:"frc-planner-dashboard/1"` and includes:

```json
{
  "schema_version":"frc-planner-dashboard/1",
  "mode":"sandbox",
  "source_kind":"synthetic_mock",
  "real_feed_implemented":false,
  "connection":{"connected":true,"stale":false,"reason":"offline synthetic sandbox"},
  "robot_us":"1000000","epoch":"1","snapshot_id":"1","obstacle_map_version":"1",
  "field":{"season":"offseason-synthetic","map_id":"dashboard-lab-8x4","geometry_revision":"1"},
  "robot":{"pose":{"x_m":1,"y_m":2,"heading_rad":0},"velocity":{"vx_mps":0,"vy_mps":0,"omega_radps":0}},
  "scenario":{},"obstacles":[],"plan":{},"errors":[]
}
```

Identity/timestamp/duration fields are **decimal strings on the wire**:
`robot_us,captured_us,estimated_robot_us,age_us,epoch,snapshot_id,obstacle_map_version,
sequence,generation,issued_us,valid_until_us,solver_duration_ns`. Inputs accept exact
nonnegative signed-long values as decimal strings or JSON integer tokens. Use
`BigInt` for exact JavaScript comparisons/differences, converting only bounded
display durations to Number. Metric values, replay index/count, path count and
budget milliseconds remain JSON numbers. The separately owned `frc-field-map/1`
document retains its numeric integer revision/image dimensions per its schema.

Scenario fields are `field`, `bounds:{min_x_m,min_y_m,max_x_m,max_y_m}`,
`start:{x_m,y_m,heading_rad,vx_mps,vy_mps,omega_radps}`,
`goal:{x_m,y_m,heading_rad,position_tolerance_m,heading_tolerance_rad,velocity_tolerance_mps}`,
`footprint:{length_m,width_m}`,
`constraints:{max_speed_mps,max_acceleration_mps2,max_angular_speed_radps}`,
`obstacles:[{id,x_m,y_m,radius_m,uncertainty_margin_m,dynamic}]`, nullable
`field_map`, computed `planning_supported` and `warnings`. Headings are CCW SI
radians, independent of travel direction. Circle dimensions/uncertainty are raw
occupancy envelopes valid for the whole offline request horizon; the backend owns
robot footprint/clearance inflation exactly once. The mock dynamic circle is a
conservative occupancy envelope, not a velocity/covariance or space-time prediction.

Plan fields include `status,generation,request_id,task_id,epoch,snapshot_id,
obstacle_map_version,field,start,goal,issued_us,valid_until_us,budget_ms,
solver_duration_ns,backend_id,detail,positions:[{x_m,y_m,heading_rad}],point_count,
display_limit_exceeded,geometry_kind:"geometric",executable:false`. IDLE has null
request/task IDs; PENDING has request metadata but no positions. Failures/expired
results carry no display geometry. Solver nanoseconds are measured computation,
never motion duration; no timed trajectory is fabricated. Poll at **at most5 Hz**,
one outstanding request at a time. No search-node stream or continuous replanning
promise is made.

Replay load accepts
`{schema_version:"frc-planner-dashboard-replay/1",source_kind:"synthetic_mock",frames:[...]}`
with 1–500 normalized frames. `GET /api/replay/sample?index=N` returns that frame
plus `replay:{frame_count,index}`; sample selection updates the local view index.
The three built-in frames and fixtures are deterministic authored recordings,
labelled `synthetic-recording/not-a-solver`; they are not benchmark measurements
or evidence of collision/robot qualification.

Live/replay frame requirements are the normalized schema/source, `session_id`,
`sequence`, `robot_us`, optional `captured_us` (default robot_us), `epoch`, snapshot
and map IDs, `field`, actual SI `robot.pose`/`robot.velocity`, optional circles and
optional plan. Non-IDLE recorded plans require all identity/time/duration/provenance
fields plus monotonic `generation`. Success requires 2–512 poses, matching frame
context and a live expiry horizon. An optional scenario must match the frame's
field and circles. Missing obstacles remain empty; missing goals remain unknown.

Sequence must increase within a session; clock rollback requires a new session
with a newer epoch. Field/snapshot/map rollback is rejected. Generation high-water
marks persist across IDLE, task replacement and recorded replay frames; a retired
request cannot reappear. A canceled/terminal request cannot regain a path at the
same generation. New sessions reset per-session generation history and clear old
geometry. Live age combines producer `robot_us-captured_us` with monotonic elapsed
time since receipt, without conflating clock epochs. At age≥1 second or expiry,
geometry is hidden and state is stale/disconnected. Rejected updates hide previous
geometry and report the reason. Default live is disconnected with null robot/field.

`SyntheticMockProducer [--port 8086] [--frames 10] [--interval-ms 200]` posts only to
127.0.0.1; bounds are1–50 frames and200–1000 ms. It authors simple SI mock poses and
recorded geometry, sends no browser headers, and makes no planner/robot/NT call.
Its epoch is an opaque session-generation identifier, not a wall-to-robot timestamp
conversion. After it stops, live age makes the display stale.

`field_map` must be an explicitly approved exact
[frc-field-map/1](field-map-format.md) document. The Java boundary independently
checks canonical digest, revision reviews, schema fields, affine determinant and
bottom row, raster metadata, fit/independent point distinctness and recomputed RMS,
finite bounded rings, winding, intersections/containment, provenance, nullable Z
range and total vertex capacity. It does not fetch image bytes or provenance URIs.
Physical outer polygons become circumscribed circles, explicitly filling obstacle
holes conservatively. Unsupported nonrectangular boundaries or boundary holes are
retained for rendering and return HTTP409 `UNSUPPORTED_FIELD_GEOMETRY` on planning.
Image/calibration approval is local review, not physical authenticity.

Implemented: bounded desktop HTTP model, offline planner gating, synthetic replay
and CLI telemetry. JDK17 executions passed 30 model checks and 12 loopback HTTP
checks, including foreign Host/Origin rejection, browser live-ingest rejection,
duplicate/trailing JSON, body limits and a slow-body connection deadline. Actual
local Chromium interactions covered edits/replanning, preview/cancel, replay
scrubbing, mock live freshness, image calibration/review/edit invalidation,
responsive layout and stopping preview after HTTP loss. See
[browser evidence](dashboard-validation.md). HTTP-server deadline
behavior on JDK25 and any real feed remain unrun. Physically verified
capabilities: none.
