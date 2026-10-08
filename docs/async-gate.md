# Asynchronous proposal gate

`pathplanning.backend.LatestRequestPlanner` wraps the exact World-State
`frc-planner/1` `PlannerBackend` SPI. It uses one daemon worker, retains at most one
active request plus one replaceable pending request, and starts disabled. A newer
submission immediately clears the old proposal and supersedes older futures with
`STALE_RESULT`. Active solver work receives a composed cancellation callback.
Even a backend that ignores that callback cannot restore an old proposal.

The constructor takes a `PlannerBackend`. Its public methods are:

- `setEnabled(boolean)`; enabling requires a fresh submission.
- `submit(Request)`, returning a diagnostic `CompletableFuture<Result>`.
- `currentProposal(long robotNowUs)`, returning `Optional<Result>`.
- `cancel()` and `invalidate(String reason)`.
- `invalidate(Context)` and `invalidate(WorldSnapshot)`.
- `close()`.

`Context` holds epoch, snapshot ID, obstacle-map version and field identity; use
`Context.from(Request)` or `Context.from(WorldSnapshot)`. Before a context is
explicitly pinned, accepted requests establish/update it. Calling
`invalidate(Context)` or `invalidate(WorldSnapshot)` pins the authoritative
context; subsequent mismatched submissions return `STALE_RESULT`. Refresh that
context before submitting work for a changed snapshot. Even an equal-context
invalidation revokes outstanding work.

World-State may already own a worker; use exactly one worker owner and call the
synchronous backend there instead of stacking asynchronous schedulers. Preserve
the original request snapshot during planning. A periodic publication identifier
alone is not a meaningful planning-context revision. The robot owner determines
when task/pose/map/sensing changes invalidate that snapshot; this gate cannot make
that domain decision and strictly rejects any changed pinned context.

Use invalidation for a source restart, pose reset, task replacement, field or
geometry change, obstacle-map version change, and snapshot replacement. Robot
disable calls `setEnabled(false)`; driver cancellation calls `cancel()`. Carry a
new epoch when the robot clock restarts. A backward `robotNowUs` read clears all
proposals and outstanding work, and remains rejected until time catches up or an
explicit new epoch resets the clock guard. A snapshot/context update within the
same epoch never hides a clock rollback.

`currentProposal` checks enable/cancel state, latest request and task IDs, epoch,
snapshot, obstacle-map version, field identity and an exclusive expiry horizon.
Both request and result must be live in robot microseconds. Search budgets use
`System.nanoTime()`; queue delay consumes the same budget. The worker checks the
budget before starting and before publishing, independent of backend compliance.
Solver elapsed nanoseconds never become a trajectory duration. Gate-generated
invalidation diagnostics report elapsed work observed at invalidation, including
zero when it had not started; an uncooperative backend may still be unwinding.

Future results are diagnostics, including a success that was valid when produced.
An immutable result retained by a caller cannot be physically revoked. The robot
execution guard must call the current-proposal gate and independently check
enable, driver/command authority, localization, sensing age, field/map context and
swept collision every control cycle. The gate has no actuator, objective selection,
match-mode transition or physical-task-success API. Never execute directly from a
future callback. Avoid doing planning in a periodic callback; `submit` only replaces
bounded bookkeeping and forwards the immutable request to the worker.

Cancellation callbacks must be nonblocking and side-effect-free; future
continuations must be nonblocking. Use
`thenAcceptAsync` (with a bounded application executor) for diagnostic consumers:
`CompletableFuture` synchronous continuations run on the thread completing it.
Success completion is linearized under the gate lock to guarantee that it cannot
publish after a newer submission. No heavy or blocking work belongs in those
continuations.

`close()` clears proposals/futures, signals cancellation, interrupts the worker and
waits at most 100 ms. A backend that never polls cancellation can delay subsequent
work and can outlive close as a daemon thread. Java cannot safely forcibly terminate
it; it still cannot publish. Use a bounded, cancellation-polling backend. No planning
frequency or controller performance is asserted by this gate.

Capability scope: implemented Java 17, controller-independent bookkeeping;
software concurrency tests cover latest-request ordering and invalidation in the
parent test harness. Controller deployment, robot execution and physical stopping
remain unverified and outside this component.
