# Local browser visualization and offline simulation plan

The confirmed first scope is a local **2D Canvas dashboard with a modest offline
kinematic preview**. The implementation uses the existing planner and a separate
dashboard projection; it does not introduce a replacement planner wire contract.
The existing `frc-planner/1` World-State API remains the domain contract, pinned
in [API provenance](../vendor/world-state-api-src/PROVENANCE.md). The
browser visualizes geometric proposals on the laptop, replays recorded
diagnostics, and edits synthetic scenarios in a separate offline mode.
Robot-code integration remains on hold. This application has no motor,
match-mode, command-authority, objective-completion, or controller-deployment API.

## Confirmed first scope

| Choice | Implemented choice | Consequence |
| --- | --- | --- |
| 2D — confirmed | HTML Canvas field view | Matches planar planner geometry; no 3D scope is selected. |
| Geometric path visualization — confirmed | Show proposals and their points/segments | Solver time never becomes motion duration. |
| Simple offline kinematic following — confirmed | Separate synthetic robot marker and simulation clock | Bounded speed/acceleration, rest at corners, and deliberate final rotation; not drivetrain physics or timed planner output. |

The sample is a conspicuously labeled, generic **8 × 4 m SYNTHETIC** laboratory,
not official FRC geometry. Sandbox editing, deterministic recorded replay and
read-only live inspection remain distinct. The actual entry point is
`pathplanning.dashboard.DashboardServer`, loopback port 8086; the current view
polls its normalized state at 5 Hz, a display setting rather than a planning-rate
claim. [Dashboard telemetry](dashboard-telemetry.md) records the implemented
routes and projection. The remaining sections preserve the design rationale;
earlier proposed SSE/routes/entry-point names are not the current server API.

The implemented client serves only local assets (`/app.js`, `/style.css`, and
`/field-import.js`). Numeric edits preserve their DOM inputs across polling,
revoke the previous proposal, and validate/save after a short pause, Tab, or
Enter. Imports reject nonfinite geometry, excessive bounds, and unsafe numeric
owner timestamps/IDs; use decimal strings for large metadata values. Replay
requests have ordering tokens, including mode changes and file loads.

Preview requires a fresh matching sandbox proposal and stops when HTTP state
ages beyond one second, the context changes, cancellation occurs, virtual expiry
arrives, or the conservative swept-footprint check fails. Each translation
segment uses a bounded triangular or trapezoidal speed profile, with zero speed
at corners; stationary heading changes use a separate angular-speed limit.
These browser calculations are a kinematic demonstration, not planner time
parameterization or drivetrain validation. Reviewed-map circles and bounds are
derived inputs and cannot be edited through the generic circle controls.

## Implemented application boundary

The optional `dashboard` source set uses JDK17 `jdk.httpserver`, Jackson2.17.2,
and packaged local HTML/CSS/JavaScript. It binds only 127.0.0.1, validates Host and
browser request origin, and exposes the exact bounded routes documented in
[dashboard telemetry](dashboard-telemetry.md). The display polls complete states;
SSE and the earlier proposed `/offline/*` routes were not implemented. No web
framework, cloud service, Node package installation or native WPILib is required.

Core and World-State adapter artifacts contain none of these desktop dependencies.
The server owns one planning worker and one latest pending request, two bounded
HTTP workers, fixed body/response limits and process-local connection/time limits.
Raster files remain in the browser; the server receives approved geometry metadata
and independently verifies its revision-bound canonical digest. Raster review
establishes local document consistency, not real field accuracy or authenticity.

## Modes and authority

Sandbox invokes the exact immutable `frc-planner/1` backend and publication gate.
Editing, canceling, replacement, expiry and changed context revoke the old proposal.
The kinematic preview is a separate browser demonstration with corner stops,
linear speed/acceleration limits, stationary angular-speed limits and conservative
footprint sweep checks. It never supplies `Result.trajectory` or task completion.

Replay samples a bounded deterministic authored recording. Live reads externally
supplied `synthetic_mock` frames through the CLI-only local ingest route. The
browser cannot ingest live frames, submit live plans or edit live goals. Source
age, expiry, monotonic identity/generation and session retirement checks hide old
geometry; display pause cannot extend validity. No real NT/owner feed is connected.
Owner `_us` remains microseconds and solver duration remains nanoseconds. Decimal
strings preserve signed-long metadata in JavaScript without numeric rounding.

The independently owned [field-map format](field-map-format.md) preserves WPILib
NWU metres, physical polygon rings/holes, nullable vertical ranges and provenance.
Perspective images require a separate supported calibration model and are rejected
here. Unknown height remains unknown. The current backend requires a rectangular
boundary without boundary holes and conservatively encloses obstacle outer rings
in circles; unsupported geometry remains visible with planning disabled. Only the
backend adds robot footprint inflation.

## Delivered validation and remaining work

Current Java17 build/packaging and CPU tests passed 18 core acceptance groups,
20 owner integration checks, 30 desktop model checks and 12 loopback HTTP checks.
Independent JavaScript field-map tests passed58 checks. Actual local Chromium
interaction verified editing/replanning, preview/cancel, replay, mock live expiry,
image calibration/review/revision invalidation, HTTP-loss stopping and mobile width.
[Browser evidence](dashboard-validation.md) records observed outcomes,
resolved intermediate failures and screenshots. [Mac performance](mac-performance.md)
provides separately recorded bounded CPU measurements and their limits.

A genuine owner/NT telemetry bridge, official field measurement, controller
packaging/runtime tests, robot execution integration, physical verification and
combined robot-stack timing require separate work. No 3D or drivetrain physics
model is supplied by this desktop application. The coordinated Custom-Vision
viewer may independently render explicitly reviewed known-height solids while
preserving the same raw field-map document.
