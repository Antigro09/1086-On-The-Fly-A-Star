package pathplanning.backend;

import java.util.Objects;
import java.util.Optional;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CompletableFuture;
import org.frcworldstate.core.Geometry.FieldIdentity;
import org.frcworldstate.core.PlannerBackend;
import org.frcworldstate.core.PlannerBackend.Request;
import org.frcworldstate.core.PlannerBackend.Result;
import org.frcworldstate.core.PlannerBackend.Status;
import org.frcworldstate.core.World.WorldSnapshot;

/**
 * One bounded worker and one replaceable pending request. This gate returns geometric proposals;
 * the robot must independently guard execution on every cycle and owns physical task completion.
 * Cancellation callbacks must be nonblocking and side-effect-free; use async future continuations.
 */
public final class LatestRequestPlanner implements AutoCloseable {
    public static final String GATE_ID = "1086-latest-request-gate/1";

    /** The authoritative snapshot identity, including the field absent from the v1 Result record. */
    public record Context(long epoch, long snapshotId, long obstacleMapVersion, FieldIdentity field) {
        public Context {
            if (epoch < 0 || snapshotId < 0 || obstacleMapVersion < 0)
                throw new IllegalArgumentException("negative context identity");
            Objects.requireNonNull(field, "field");
        }
        public static Context from(Request request) {
            Objects.requireNonNull(request, "request");
            return new Context(request.epoch(), request.snapshotId(), request.obstacleMapVersion(), request.field());
        }
        public static Context from(WorldSnapshot snapshot) {
            Objects.requireNonNull(snapshot, "snapshot");
            return new Context(snapshot.epoch(), snapshot.id(), snapshot.obstacleMapVersion(), snapshot.field());
        }
    }

    private final Object lock = new Object();
    private final PlannerBackend backend;
    private final Thread worker;
    private boolean enabled;
    private boolean closed;
    private long generation;
    private long lastRobotNowUs = -1;
    private Context context;
    private boolean contextPinned;
    private Ticket active;
    private Ticket pending;
    private Ticket latest;
    private Result proposal;

    public LatestRequestPlanner(PlannerBackend backend) {
        this.backend = Objects.requireNonNull(backend, "backend");
        worker = new Thread(this::work, "1086-geometric-planner");
        worker.setDaemon(true);
        worker.start();
    }

    /**
     * Submits geometry work without running the solver on the caller's thread. The future is only
     * diagnostic; currentProposal is the guarded read. Every accepted request replaces the older
     * proposal and pending work. With no explicitly pinned context, the request establishes context.
     */
    public CompletableFuture<Result> submit(Request request) {
        Objects.requireNonNull(request, "request");
        List<Revoked> revoked;
        Ticket ticket;
        synchronized (lock) {
            if (closed || !enabled)
                return completed(request, Status.CANCELLED, closed ? "Planner is closed" : "Planner is disabled");
            Context requested = Context.from(request);
            if (contextPinned && !requested.equals(context))
                return completed(request, Status.STALE_RESULT, "Request differs from authoritative context");
            boolean newClockEpoch = context == null || context.epoch() != requested.epoch();
            revoked = revoke(Status.STALE_RESULT, "Superseded by a newer request");
            context = requested;
            if (newClockEpoch) lastRobotNowUs = -1;
            ticket = new Ticket(request, generation);
            latest = ticket;
            pending = ticket;
            lock.notifyAll();
        }
        completeRevoked(revoked);
        return ticket.future;
    }

    /**
     * Returns only the current successful proposal while robot time and all identity/freshness
     * guards agree. Expiry is exclusive. A robot-clock rollback revokes outstanding work; an
     * explicit new epoch is required before the lower clock can be accepted again.
     */
    public Optional<Result> currentProposal(long robotNowUs) {
        if (robotNowUs < 0) throw new IllegalArgumentException("negative robot timestamp");
        List<Revoked> revoked = List.of();
        Optional<Result> current = Optional.empty();
        synchronized (lock) {
            if (lastRobotNowUs >= 0 && robotNowUs < lastRobotNowUs) {
                revoked = revoke(Status.STALE_RESULT, "Robot clock rolled backward; invalidate with a new epoch");
            } else {
                lastRobotNowUs = robotNowUs;
                if (!closed && enabled && latest != null) {
                    if (isCancelled(latest))
                        revoked = revoke(Status.CANCELLED, "Current request was cancelled");
                    else if (robotNowUs >= latest.original.validUntilUs())
                        revoked = revoke(Status.STALE_RESULT, "Current request expired in robot-clock domain");
                    else if (proposal != null && robotNowUs >= latest.original.issuedUs()) {
                        if (proposal.status() != Status.SUCCESS || !matches(proposal, latest.original)
                                || !Context.from(latest.original).equals(context)
                                || robotNowUs >= proposal.validUntilUs())
                            revoked = revoke(Status.STALE_RESULT, "Proposal identity or validity no longer matches");
                        else current = Optional.of(proposal);
                    }
                }
            }
        }
        completeRevoked(revoked);
        return current;
    }

    /** Initially disabled. Enabling never revives a previous request; submit new work explicitly. */
    public void setEnabled(boolean enabled) {
        List<Revoked> revoked = List.of();
        synchronized (lock) {
            if (closed && enabled) throw new IllegalStateException("Planner is closed");
            if (this.enabled != enabled) {
                this.enabled = enabled;
                revoked = revoke(Status.CANCELLED, enabled ? "Fresh request required after enable" : "Planner disabled");
            }
        }
        completeRevoked(revoked);
    }

    /** Immediately removes proposals and pending work, and signals active solver cancellation. */
    public void cancel() {
        List<Revoked> revoked;
        synchronized (lock) { revoked = revoke(Status.CANCELLED, "Planner cancelled"); }
        completeRevoked(revoked);
    }

    /**
     * Pins the robot's authoritative context and invalidates all work, even for an equal context.
     * Call for snapshot/map/field changes, source restart, pose reset or task replacement before
     * submitting a request. Only a changed epoch resets the robot-clock rollback guard.
     */
    public void invalidate(Context context) {
        Objects.requireNonNull(context, "context");
        List<Revoked> revoked;
        synchronized (lock) {
            boolean newClockEpoch = this.context == null || this.context.epoch() != context.epoch();
            revoked = revoke(Status.STALE_RESULT, "Authoritative context invalidated");
            this.context = context;
            contextPinned = true;
            if (newClockEpoch) lastRobotNowUs = -1;
        }
        completeRevoked(revoked);
    }

    public void invalidate(WorldSnapshot snapshot) { invalidate(Context.from(snapshot)); }

    /** Invalidates work while retaining the current context and robot-clock history. */
    public void invalidate(String reason) {
        if (reason == null || reason.isBlank()) throw new IllegalArgumentException("missing invalidation reason");
        List<Revoked> revoked;
        synchronized (lock) { revoked = revoke(Status.STALE_RESULT, reason); }
        completeRevoked(revoked);
    }

    /**
     * Invalidates all proposals/futures, interrupts the daemon worker and waits at most 100 ms.
     * Java cannot forcibly stop an uncooperative backend; it can never publish after this call.
     */
    @Override public void close() {
        List<Revoked> revoked;
        synchronized (lock) {
            if (closed) return;
            closed = true;
            enabled = false;
            revoked = revoke(Status.CANCELLED, "Planner closed");
            lock.notifyAll();
        }
        completeRevoked(revoked);
        worker.interrupt();
        if (Thread.currentThread() != worker) {
            try { worker.join(100); }
            catch (InterruptedException interrupted) { Thread.currentThread().interrupt(); }
        }
    }

    private void work() {
        while (true) {
            Ticket ticket;
            synchronized (lock) {
                while (!closed && pending == null) {
                    try { lock.wait(); }
                    catch (InterruptedException interrupted) { if (closed) return; }
                }
                if (closed) return;
                ticket = pending;
                pending = null;
                active = ticket;
                ticket.startedNanos = System.nanoTime();
                ticket.started = true;
            }
            Result result;
            try {
                if (isCancelled(ticket)) result = failure(ticket, Status.CANCELLED, "Cancelled before solver start");
                else if (ticket.original.budget().expired(System.nanoTime()))
                    result = failure(ticket, Status.TIMEOUT, "Budget expired while waiting for worker");
                else {
                    result = backend.plan(ticket.forwarded);
                    if (result == null) result = failure(ticket, Status.NO_PATH, "Backend returned no result");
                }
            } catch (RuntimeException failure) {
                result = failure(ticket, Status.NO_PATH, "Backend failed: " + failure.getClass().getSimpleName());
            }
            synchronized (lock) {
                if (active == ticket) active = null;
                if (ticket.future.isDone()) continue;
                if (ticket.revocation != null) result = ticket.revocation;
                else if (closed || !enabled || isCancelled(ticket))
                    result = failure(ticket, Status.CANCELLED, "Cancelled before publication");
                else if (ticket.generation != generation || latest != ticket
                        || !Context.from(ticket.original).equals(context) || !matches(result, ticket.original))
                    result = failure(ticket, Status.STALE_RESULT, "Result provenance no longer matches latest request");
                else if (ticket.original.budget().expired(System.nanoTime()))
                    result = failure(ticket, Status.TIMEOUT, "Solver exceeded monotonic search budget");
                else if (result.validUntilUs() <= ticket.original.issuedUs()
                        || (lastRobotNowUs >= 0 && lastRobotNowUs >= result.validUntilUs()))
                    result = failure(ticket, Status.STALE_RESULT, "Result expired before publication");
                if (result.status() == Status.SUCCESS) proposal = result;
                // Completing under the same lock linearizes success before any newer submission.
                // Callers must use nonblocking/async continuations so callbacks do not hold the gate.
                ticket.future.complete(result);
            }
        }
    }

    /** Caller holds lock. At most an active and a pending ticket are retained. */
    private List<Revoked> revoke(Status status, String reason) {
        generation++;
        proposal = null;
        latest = null;
        List<Revoked> revoked = new ArrayList<>(2);
        if (active != null) cancelTicket(active, status, reason, revoked);
        if (pending != null) cancelTicket(pending, status, reason, revoked);
        pending = null;
        return revoked;
    }

    private void cancelTicket(Ticket ticket, Status status, String reason, List<Revoked> revoked) {
        ticket.cancelled = true;
        if (!ticket.future.isDone() && ticket.revocation == null) {
            ticket.revocation = failure(ticket, status, reason
                    + (ticket.started ? "; diagnostic elapsed measured at invalidation; solver may still unwind" : ""));
            revoked.add(new Revoked(ticket, ticket.revocation));
        }
    }

    /** Complete diagnostics after the mutation, so continuations cannot reenter half-updated state. */
    private static void completeRevoked(List<Revoked> revoked) {
        for (Revoked completion : revoked) completion.ticket.future.complete(completion.result);
    }

    private record Revoked(Ticket ticket, Result result) {}

    private static boolean matches(Result result, Request request) {
        return result.requestId().equals(request.requestId()) && result.taskId().equals(request.taskId())
                && result.epoch() == request.epoch() && result.snapshotId() == request.snapshotId()
                && result.obstacleMapVersion() == request.obstacleMapVersion()
                && result.validUntilUs() <= request.validUntilUs();
    }

    private static boolean isCancelled(Ticket ticket) {
        if (ticket.cancelled || ticket.future.isCancelled()) return true;
        try { return ticket.original.cancellation().cancelled(); }
        catch (RuntimeException invalidCallback) { return true; }
    }

    private static Result failure(Ticket ticket, Status status, String detail) {
        long duration = ticket.started ? Math.max(0, System.nanoTime() - ticket.startedNanos) : 0;
        return failure(ticket.original, status, duration, detail);
    }

    private static Result failure(Request request, Status status, long duration, String detail) {
        return new Result(request.requestId(), request.taskId(), request.epoch(), request.snapshotId(),
                request.obstacleMapVersion(), status, null, null, duration, request.validUntilUs(), GATE_ID, detail);
    }

    private static CompletableFuture<Result> completed(Request request, Status status, String detail) {
        return CompletableFuture.completedFuture(failure(request, status, 0, detail));
    }

    private static final class Ticket {
        final Request original;
        final Request forwarded;
        final long generation;
        final CompletableFuture<Result> future = new CompletableFuture<>();
        volatile boolean cancelled;
        volatile Result revocation;
        volatile boolean started;
        volatile long startedNanos;

        Ticket(Request request, long generation) {
            original = request;
            this.generation = generation;
            forwarded = new Request(request.requestId(), request.taskId(), request.epoch(), request.snapshotId(),
                    request.obstacleMapVersion(), request.field(), request.startPose(), request.startVelocity(),
                    request.goal(), request.footprint(), request.constraints(), request.bounds(), request.obstacles(),
                    request.issuedUs(), request.validUntilUs(), request.budget(), () -> isCancelled(this));
        }
    }
}
