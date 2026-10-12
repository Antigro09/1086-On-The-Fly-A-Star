# Curves and bounded motion proposals

`CurvedMotionPlanner` adds collision-certified cubic Bézier geometry and a separate
kinematic speed profile. The desktop Sandbox uses this planner through its single
`LatestRequestPlanner` worker. Refresh the local page, select Sandbox, then Plan;
the curve, speed chart and preview come from the actual Java result. Replay and
Live retain their recorded geometry and acquire no fabricated curve or timing.
Robot following and actual robot-code integration remain on hold.

## Search, curves and objective

The bounded eight-connected A* search supplies route seeds at three clearance
settings: 0.02, 0.10 and 0.25 m. It determines route topology, not final controls.
Legacy certified corner candidates provide an incumbent. `SplineOptimizer`
then proposes fewer detour portals, solves a globally coupled natural/clamped
cubic spline and moves its interior anchors in continuous space. The whole
spline is recomputed after each trial. A shared chord-length parameter yields
continuous tangent and geometric curvature; the initial measured travel direction
is clamped when its speed is nonzero. End positions are exact, including a
near-duplicate terminal waypoint.

Detour coalescing and polyline simplification only propose anchors; they do not
establish that a shortcut is safe. Deterministic coordinate descent evaluates at
most 320 sampled proxies per route with at most 48 anchors. The proxy combines
traversal time, bending energy and an obstacle/edge clearance penalty. Its weights
are 0.035 s·m for integrated squared curvature and 5,000 s/m³ for integrated
squared clearance violation. Sampling guides optimization only. Each candidate
must receive a fresh exact-controls collision/curvature certificate, canonical
arc coordinates and the actual constrained motion profile before selection.
The proxy never grants publication authority. Maps beyond 512 raw obstacle boxes
skip this optional refinement; the original solver still checks the complete map
and preserves its conservative fallback.

Certified candidates are profiled immediately; duplicate controls are skipped.
The minimum actual-duration feasible candidate before the shared refinement
cutoff wins. Refinement uses 80% of the original monotonic budget, reserving the
remainder for conversion and final validation. Cancellation or the actual deadline
overrides every incumbent.

The objective is **motion duration**, not waypoint count, geometric length or
solver time. This is an anytime comparison of a bounded candidate set, not a
proof of a global optimum. Clear straight routes remain straight. Broad detour
spans curve throughout their legs rather than adding small fillets between
collinear grid edges. When whole-route refinement cannot be certified within the
budget, the legacy candidate remains explicitly identified; its piecewise
straight fallback stops at actual tangent discontinuities.
Safe paths can remain conservative or be rejected because the disk footprint,
circle-to-box envelopes, grid and finite candidate set limit completeness.

[PathPlanner's on-the-fly guide](https://pathplanner.dev/pplib-pathfinding.html)
informed obstacle-aware route generation and separate path constraints.
[Choreo](https://choreo.autos/) informed optimizing traversal time with coupled
constraints. This implementation uses a point-mass model; it does not implement
Choreo's drivetrain force/motor optimization. [AD*](https://www.cs.cmu.edu/~ggordon/likhachev-etal.anytime-dstar.pdf)
can reuse search work when maps change. The existing bounded A* was retained
because changing the search alone would not solve corner geometry or timing, and
there is no controller measurement showing that a stateful cache beats fresh
search here. A future AD* backend must preserve immutable request identity,
invalidation and collision checks; stale cached occupancy cannot gain authority.

The [Choi/Elkaim Bézier paper](https://users.soe.ucsc.edu/~elkaim/Documents/BezierWCES08.pdf)
informed optimizing complete low-degree spans with coupled derivatives and
collision constraints. This bounded spline refinement does not reproduce its
vehicle model or nonlinear solver. The [pinned PathPlanner GUI optimizer](https://github.com/mjansen4857/pathplanner/blob/e02bbf3176588166e8fe5192ab8dea85f6d62f7a/lib/util/path_optimizer.dart)
varies handles/headings with trajectory-duration fitness; its desktop genetic
search is not adopted as a controller loop or treated as collision proof.

## Collision and motion constraints

Raw obstacle dimensions, age, uncertainty and bounded motion belong to the
World-State envelope builder. This backend adds the raw robot's circumscribed
radius plus clearance exactly once. Adaptive Bézier subdivision bounds the
whole control hull relative to its chord; swept capsule tests then cover the
entire curve and footprint against bounds, cells and static/dynamic envelopes.
After global-coordinate conversion, the actual controls are certified again
with a numerical roundoff guard. Center samples alone are never the certificate.
Conservative disk geometry covers robot rotation and goal tolerance regions.

The timing stage recomputes canonical arc length using a bounded integrator.
Each interval receives a conservative curvature bound verified independently
from its controls. Forward/backward reachable-speed passes anticipate braking
and maximize the fixed conservative velocity envelope. They enforce maximum
speed, a separate centripetal acceleration cap and **total linear acceleration**:
`sqrt(tangentialAcceleration² + (curvature * speed²)²) <= maxAcceleration`.
Reported `acceleration_mps2` is signed tangential acceleration;
`total_acceleration_mps2` is its norm combined with the normal component.
The current velocity direction fixes the first tangent, preserving strafe and
reverse travel independently of holonomic heading. Infeasible boundary speeds
are rejected instead of silently lowered.

Holonomic rotation uses a separate bounded angular profile preserving current
angular velocity and stopping at the requested heading. The wrapped turn and
adjacent full-turn alternatives are considered. When angular motion takes
longer, a stopped linear endpoint permits holding there: a moving start/stopped
end finishes translation first; a stopped start/moving end delays departure.
Unsupported longer synchronization with both linear endpoints moving is
explicitly rejected. Stationary and in-place paths are supported.

These limits do not model swerve module velocity/force, motor voltage, traction,
jerk or feedforward. Acceleration can change discontinuously at phase boundaries.
Global cubic candidates remove geometric curvature jumps at their regular joins;
their local `[0,1]` parameter derivative magnitudes can differ with span lengths.
Continuity tests normalize derivatives by the common chord parameter rather than
incorrectly requiring equal raw local derivatives.
Actual drivetrain parameters and independent physical validation are required
before a follower can treat this point-mass proposal as executable motion.

All stages poll cancellation and monotonic elapsed time. Search caps cells,
expansions and queues; smoothing caps 128 segments, 1,024 knots, depth 16 and
16,384 subdivision nodes. Timing additionally bounds subdivisions/passes and
transport accepts at most 768 samples. Failure returns typed status and no
curve/profile. No per-candidate budget reset grants additional total time.

## API, validity and moving destinations

The owner `frc-planner/1` API and hashes remain unchanged.
`AStarPlannerBackend` continues to implement the geometric SPI.
`CurvedMotionPlanner.plan(request, extraLimits)` returns a separate `Proposal`
containing the original identity-bearing geometric result, immutable curve,
profile, candidate count, robot-domain planned start, validated prefix and
measured planning nanoseconds. `Result.trajectory()` remains null. The optional
implementation values in `pathplanning.motion` are not a replacement owner SPI.
Only one worker should own scheduling.

`extraLimits` supplies centripetal acceleration, angular acceleration and final
linear speed. The final speed must fit both maximum speed and the owner's goal
velocity tolerance. No goal tolerance is silently changed. The full-plan method
requires the entire motion to fit the **remaining** request/envelope horizon at
publication, including computation time. `planRecedingHorizon` instead marks only
the prefix before expiry as forecast-qualified; its later suffix is a proposal
for subsequent replans. It grants no authority beyond `validUntilUs`. Full-plan
mode is used for the offline desktop preview.

A static waypoint or moving game piece can cause a new immutable goal request.
World-State owns persistent track identity, selecting the closest **feasible**
piece, coasting across missed frames, loss-of-vision policy and bounded
approach/interception prediction. It should place the approach pose outside the
piece's physical collision envelope rather than removing the obstacle to make
its occupied center reachable. Selecting/chasing a piece and declaring pickup
success are not implemented in this backend.

Meaningful target updates replace the pending request; an older request cannot
publish after replacement. Pass actual current pose/linear/angular velocity into
each replan. Coalesce incidental frame publications externally so unchanged
goals do not continually cancel useful work. The robot guard must check pose
drift, target displacement, epoch, task, map and sensing/expiry each cycle;
disable/cancel clears proposals immediately. Replanning is not a stop mechanism.
Envelopes cover the entire qualified interval conservatively; this is not
space-time collision scheduling or permission to cross a future occupied region.

## Desktop wire extension

Scenario `motion_limits` contains `max_centripetal_acceleration_mps2`,
`max_angular_acceleration_radps2` and `goal_end_speed_mps`. On a current Sandbox
SUCCESS only, `plan.motion` carries `geometry_kind:"cubic_bezier"`,
`executable:false`, `duration_s`, `length_m`, `peak_speed_mps`, `candidate_count`,
`planned_start_us`, `validated_prefix_s`, detail, cubic controls `p0`–`p3`,
per-segment arc coordinates and time samples. `planned_start_us` is an exact
decimal string, as are the existing timestamps and solver nanoseconds.

The Canvas draws the exported controls with `bezierCurveTo`. Playback integrates
the backend speed/angular intervals and inverts the same curve's arc coordinate;
it does not linearly shortcut bends or stop at every exported point. Pause and
playback rate affect display only. Real monotonic elapsed time still advances
request expiry, including between HTTP polls; cancel, stale data and context
replacement retire the preview. The UI keeps solver milliseconds separate from
motion seconds and labels the proposal as non-executable.
An optional **Curve controls** checkbox draws only returned anchors, handles and
control polygons. It fits nothing in the browser. The default view emphasizes
the actual cubic and hides its diagnostic A* seed overlay. Stale styling applies
to both geometry and handles.

## Executed checks and remaining target work

Run from this checkout with JDK17, Gradle8.5, Node24.14.0 and one worker:

```sh
./gradlew --offline --no-daemon --max-workers=1 --gradle-user-home ../.gradle-user \
  build worldStateAdapterJar dashboardContractCheck dashboardHttpCheck
node fixtures/dashboard/test-motion-preview.mjs
node fixtures/dashboard/test-curve-controls.mjs
node fixtures/dashboard/test-scenario-import.mjs
node fixtures/field-map/test-field-map.mjs
node --check src/dashboard/resources/dashboard/app.js
node --check src/dashboard/resources/dashboard/motion-preview.js
python3 tools/verify_jdk25.py --java-home "$JDK25_HOME" --release 17
# Supply an actual locally captured successful Sandbox /api/state response:
node fixtures/dashboard/test-generated-motion.mjs PATH_TO_SANDBOX_STATE_JSON
```

The common core and adapter compile under strict `-Xlint:all -Werror` and run on
Temurin17.0.20.1+1 and Temurin25.0.4.1+1, with Java17 bytecode. Passed: 18 existing
planner groups, 37,019 Bézier assertions, nine spline groups / 9,796 checks,
16 timing groups, 5,345 motion integration checks,
20 owner-envelope/adapter/validator checks, 35 desktop model checks,
12 loopback HTTP checks, 13 motion-preview checks, eight scenario-import checks
and 58 field-map checks. Exact-controls UI rendering also passes its independent
handler test. Motion tests cover obstacles, corner cutting, converted
controls, curvature/collision regressions, dense total-acceleration checks,
braking/acceleration, strafe/reverse, endpoint velocities, in-place rotation,
initial angular velocity, horizon expiry, cancellation, and a blocked older
request replaced by a moving goal before publication. A real HTTP result was
also consumed by the actual JavaScript evaluator at 241 interpolated times,
checking curve positions, endpoint states, speed and total acceleration. The
earlier default scenario produced five cubic segments, four feasible candidates,
4.711728566 s of motion and peak speed 2 m/s. Its measured solver duration
22.75975 ms is one Mac smoke observation, not a distribution or controller claim.
Current automated preview
checks are CPU/model/HTTP/JavaScript evidence; earlier browser screenshots cover
the preceding polyline implementation, not the new curved interaction.

Immediate target remains WPILib2026.2.2/JDK17 roboRIO. The separate Systemcore
target is WPILib2027.0.0-alpha-7/JDK25; WPILib2026+Systemcore remains unsupported
by the [official testing matrix](https://github.com/wpilibsuite/SystemcoreTesting).
This component has no WPILib/NT dependency and keeps all JSON `_us` in
microseconds. Host packaging and tests do not establish controller latency.
The earlier Mac benchmark measures geometric search; it does not measure this
curve/timing stage. Use the [unrun target benchmark procedure](benchmark-procedure.md)
with representative moving targets and dynamic envelopes, recording p50/p95/p99,
max, deadline misses, memory and combined robot load. No new heavy benchmark,
controller deployment, drivetrain following or physical success test was run.

## Whole-route correction evidence

The earlier 31-cubic gauntlet was not an adequate smooth-path example: 75.5% of
its arc length was collinear, its corner deviations were only 1–5 cm, and its
acceptance check only required any non-collinear segment. The reproducible
26-obstacle [scenario](../fixtures/dashboard/curve-gauntlet.json) now tests
whole-route shape, curvature continuity, independently scanned occupancy and
total acceleration. Shape quotas apply to this detour fixture, not arbitrary
straight requests.

On identical geometry/footprint/constraints, the corrected proposal has nine
globally coupled spans, 14.832823 m length and 8.814200 s motion versus 31 spans,
15.304338 m and 13.392327 s previously: 34.18% lower modeled traversal time.
Every new span deviates by at least 8 cm from its chord; near-straight length at
the 2 cm threshold is zero. Signed-curvature jumps are within numerical roundoff.
These are synthetic geometry/motion comparisons, not robot benchmarks.
The corrected planner also succeeded in a separate 250 ms desktop budget smoke
test. Controller budget/memory/combined-stack qualification remains unrun.
