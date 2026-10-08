package pathplanning.dashboard;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import java.util.ArrayList;
import java.util.ArrayDeque;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;
import java.util.function.LongSupplier;
import org.frcworldstate.core.Geometry.*;
import org.frcworldstate.core.PlannerBackend.*;
import pathplanning.backend.AStarPlannerBackend;
import pathplanning.backend.LatestRequestPlanner;
import static pathplanning.dashboard.JsonSupport.*;

/** Independent sandbox, replay and read-only synthetic telemetry stores; no robot transport. */
final class DashboardModel implements AutoCloseable {
    private static final long MAX_ID = Long.MAX_VALUE;
    private static final long LIVE_STALE_US = 1_000_000;
    private final LongSupplier nanoTime;
    private final long clockOriginNs;
    private final LatestRequestPlanner planner;
    private ObjectNode scenario = defaultScenario();
    private long generation;
    private long snapshotId = 1;
    private long mapVersion = 1;
    private PlanRecord sandboxPlan;
    private List<ObjectNode> replay;
    private int replayIndex;
    private ObjectNode live;
    private long liveReceivedNs;
    private final ArrayDeque<String> previousSessions = new ArrayDeque<>();
    private String lastLiveRejection = "No producer connected; real feed is not implemented";
    private long livePlanGeneration = -1;
    private String liveRequestId;
    private boolean liveRequestRevoked;

    DashboardModel() { this(System::nanoTime); }
    DashboardModel(LongSupplier nanoTime) {
        this.nanoTime = nanoTime;
        clockOriginNs = nanoTime.getAsLong();
        planner = new LatestRequestPlanner(new AStarPlannerBackend(AStarPlannerBackend.Options.conservativeDefault(),
                nanoTime, this::robotNowUs));
        planner.setEnabled(true);
        replay = builtInReplay();
    }

    long robotNowUs() { return Math.max(1, (nanoTime.getAsLong() - clockOriginNs) / 1000 + 1); }

    synchronized ObjectNode state(String mode) {
        return switch (mode) {
            case "sandbox" -> sandboxState();
            case "replay" -> replayState(replayIndex);
            case "live" -> liveState();
            default -> throw bad("mode must be sandbox, replay or live");
        };
    }

    synchronized ObjectNode updateScenario(JsonNode patch) {
        object(patch, "scenario");
        if (patch.has("scenario")) patch = object(patch.get("scenario"), "scenario");
        ObjectNode updated = merge(scenario, object(patch, "scenario"));
        scenario = normalizeScenario(updated);
        generation++;
        snapshotId++;
        mapVersion++;
        sandboxPlan = null;
        planner.invalidate(new LatestRequestPlanner.Context(1, snapshotId, mapVersion, fieldIdentity(scenario.get("field"))));
        return sandboxState();
    }

    synchronized ObjectNode plan(JsonNode options) {
        object(options, "plan options");
        if (!scenario.path("planning_supported").asBoolean()) throw new UnsupportedGeometry();
        long budgetMs = optionalInteger(options, "budget_ms", 250, 1, 1000);
        long validityMs = optionalInteger(options, "valid_for_ms", 30000, 1, 60000);
        long issuedUs = robotNowUs();
        long expiryUs = Math.addExact(issuedUs, validityMs * 1000);
        generation++;
        String requestId = "sandbox-request-" + generation;
        String taskId = optionalText(options, "task_id", "sandbox-task", 128);
        JsonNode s = scenario.get("start"), g = scenario.get("goal"), f = scenario.get("footprint"),
                c = scenario.get("constraints"), b = scenario.get("bounds");
        List<Obstacle> obstacles = new ArrayList<>();
        for (JsonNode o : scenario.withArray("obstacles")) obstacles.add(new Obstacle(o.get("id").textValue(),
                new Vec2(o.get("x_m").doubleValue(), o.get("y_m").doubleValue()),
                o.get("radius_m").doubleValue(), o.get("uncertainty_margin_m").doubleValue(), expiryUs,
                o.get("dynamic").booleanValue()));
        Request request = new Request(requestId, taskId, 1, snapshotId, mapVersion, fieldIdentity(scenario.get("field")),
                pose(s), new Velocity2(new Vec2(s.get("vx_mps").doubleValue(), s.get("vy_mps").doubleValue()), s.get("omega_radps").doubleValue()),
                new Goal(pose(g), g.get("position_tolerance_m").doubleValue(), g.get("heading_tolerance_rad").doubleValue(), g.get("velocity_tolerance_mps").doubleValue()),
                new Footprint(f.get("length_m").doubleValue(), f.get("width_m").doubleValue()),
                new Constraints(c.get("max_speed_mps").doubleValue(), c.get("max_acceleration_mps2").doubleValue(), c.get("max_angular_speed_radps").doubleValue()),
                new Bounds(b.get("min_x_m").doubleValue(), b.get("min_y_m").doubleValue(), b.get("max_x_m").doubleValue(), b.get("max_y_m").doubleValue()),
                obstacles, issuedUs, expiryUs, new SearchBudget(nanoTime.getAsLong(), budgetMs * 1_000_000), () -> false);
        planner.invalidate(LatestRequestPlanner.Context.from(request));
        PlanRecord record = new PlanRecord(request, generation, budgetMs);
        sandboxPlan = record;
        // Constant-time volatile assignment only: never acquire the model lock in a gate continuation.
        planner.submit(request).thenAccept(result -> record.result = result);
        return sandboxState();
    }

    synchronized ObjectNode cancel() {
        generation++;
        if (sandboxPlan != null) sandboxPlan.overrideStatus = "CANCELLED";
        planner.cancel();
        return sandboxState();
    }

    private ObjectNode sandboxState() {
        long nowUs = robotNowUs();
        boolean current = planner.currentProposal(nowUs).isPresent();
        ObjectNode state = base("sandbox", "offline synthetic sandbox", true, false, nowUs, 1, snapshotId, mapVersion);
        state.set("scenario", scenario.deepCopy()); state.set("field", scenario.get("field").deepCopy());
        state.set("robot", robot(scenario.get("start")));
        state.set("obstacles", scenario.get("obstacles").deepCopy());
        state.set("plan", sandboxPlan == null ? idlePlan(generation) : viewPlan(sandboxPlan, current));
        state.set("errors", scenario.get("warnings").deepCopy());
        if (state.path("plan").path("display_limit_exceeded").asBoolean())
            state.withArray("errors").add("DISPLAY_LIMIT_EXCEEDED: validated geometry exceeds 512 points; no vertices were dropped or straightened");
        return state;
    }

    private ObjectNode viewPlan(PlanRecord record, boolean current) {
        Request r = record.request; Result result = record.result;
        String status = record.overrideStatus != null ? record.overrideStatus : result == null ? "PENDING" : result.status().name();
        if ("SUCCESS".equals(status) && !current) status = "STALE_RESULT";
        ObjectNode plan = idlePlan(record.generation).put("status", status).put("request_id", r.requestId()).put("task_id", r.taskId())
                .put("epoch", r.epoch()).put("snapshot_id", r.snapshotId()).put("obstacle_map_version", r.obstacleMapVersion())
                .put("issued_us", r.issuedUs()).put("valid_until_us", result == null ? r.validUntilUs() : result.validUntilUs())
                .put("budget_ms", record.budgetMs);
        plan.set("field", scenario.get("field").deepCopy());
        plan.set("start", scenario.get("start").deepCopy()); plan.set("goal", scenario.get("goal").deepCopy());
        if (result != null) {
            plan.put("solver_duration_ns", result.solverDurationNanos()).put("backend_id", result.backendId()).put("detail", result.detail());
            if ("SUCCESS".equals(status) && result.path() != null) {
                boolean over = result.path().points().size() > MAX_POINTS;
                plan.put("display_limit_exceeded", over).put("point_count", result.path().points().size());
                if (!over) for (Pose2 p : result.path().points()) plan.withArray("positions").add(JsonSupport.pose(p.position().x(), p.position().y(), p.headingRad()));
            }
        }
        return plan;
    }

    synchronized ObjectNode loadReplay(JsonNode document) {
        object(document, "replay");
        if (!"frc-planner-dashboard-replay/1".equals(text(document, "schema_version", 64))
                || !"synthetic_mock".equals(text(document, "source_kind", 64))) throw bad("Replay must use the synthetic replay schema");
        ArrayNode frames = list(document, "frames", 500);
        if (frames.isEmpty()) throw bad("Replay needs at least one frame");
        List<ObjectNode> normalized = new ArrayList<>(); ObjectNode previous = null;
        long highGeneration = -1; String currentRequest = null; boolean currentRevoked = false;
        for (JsonNode frame : frames) {
            ObjectNode next = normalizeFrame(frame);
            if (previous != null) validateProgress(previous, next, true);
            if (previous == null || !previous.get("session_id").equals(next.get("session_id"))) {
                highGeneration = -1; currentRequest = null; currentRevoked = false;
            }
            JsonNode plan = next.get("plan");
            if (plan.hasNonNull("request_id")) {
                long nextGeneration = plan.get("generation").longValue(); String nextRequest = plan.get("request_id").textValue();
                validatePlanGeneration(highGeneration, currentRequest, currentRevoked, nextGeneration, nextRequest, plan.path("status").asText());
                if (!nextRequest.equals(currentRequest)) currentRevoked = false;
                highGeneration = nextGeneration; currentRequest = nextRequest;
                if (terminalFailure(plan.path("status").asText())) currentRevoked = true;
            } else { highGeneration = Math.max(highGeneration, plan.get("generation").longValue()); currentRequest = null; currentRevoked = false; }
            normalized.add(next); previous = next;
        }
        replay = List.copyOf(normalized); replayIndex = 0;
        return replayState(0);
    }

    synchronized ObjectNode replaySample(int index) {
        if (index < 0 || index >= replay.size()) throw bad("Replay index outside recorded frames");
        replayIndex = index;
        return replayState(index);
    }

    private ObjectNode replayState(int index) {
        ObjectNode state = replay.get(index).deepCopy(); state.put("mode", "replay");
        state.set("connection", object().put("connected", true).put("stale", false).put("reason", "deterministic synthetic recording; no planner calls"));
        state.set("replay", object().put("frame_count", replay.size()).put("index", index));
        return state;
    }

    synchronized ObjectNode ingestLive(JsonNode frame) {
        ObjectNode next;
        try {
            next = normalizeFrame(frame);
            if (next.get("robot_us").longValue() - next.get("captured_us").longValue() >= LIVE_STALE_US)
                throw bad("Incoming telemetry is already stale in its producer clock");
            if (live != null) {
                validateProgress(live, next, false);
                String session = next.get("session_id").textValue();
                if (!session.equals(live.get("session_id").textValue()) && previousSessions.contains(session))
                    throw bad("Previously retired producer session cannot become current again");
                if (session.equals(live.get("session_id").textValue())) {
                    JsonNode plan = next.get("plan"); long incomingGeneration = plan.get("generation").longValue();
                    String incomingRequest = plan.hasNonNull("request_id") ? plan.get("request_id").textValue() : null;
                    if (incomingRequest != null) validatePlanGeneration(livePlanGeneration, liveRequestId, liveRequestRevoked,
                            incomingGeneration, incomingRequest, plan.path("status").asText());
                }
            }
        } catch (IllegalArgumentException rejected) { lastLiveRejection = rejected.getMessage(); throw rejected; }
        if (live != null && !next.get("session_id").textValue().equals(live.get("session_id").textValue())) {
            previousSessions.addLast(live.get("session_id").textValue());
            while (previousSessions.size() > 16) previousSessions.removeFirst();
            livePlanGeneration = -1; liveRequestId = null; liveRequestRevoked = false;
        }
        live = next; liveReceivedNs = nanoTime.getAsLong(); lastLiveRejection = "";
        JsonNode nextPlan = next.get("plan");
        if (nextPlan.hasNonNull("request_id")) {
            String requestId = nextPlan.get("request_id").textValue();
            if (!requestId.equals(liveRequestId)) liveRequestRevoked = false;
            livePlanGeneration = nextPlan.get("generation").longValue(); liveRequestId = requestId;
            if (terminalFailure(nextPlan.path("status").asText())) liveRequestRevoked = true;
        } else { livePlanGeneration = Math.max(livePlanGeneration, nextPlan.get("generation").longValue()); liveRequestId = null; liveRequestRevoked = false; }
        return liveState();
    }

    private ObjectNode liveState() {
        if (live == null) {
            ObjectNode state = base("live", lastLiveRejection, false, true, 0, 0, 0, 0);
            state.putNull("session_id"); state.putNull("sequence"); state.putNull("field"); state.putNull("robot");
            state.set("scenario", object().set("bounds", defaultScenario().get("bounds")));
            ((ObjectNode)state.get("scenario")).set("obstacles", array());
            state.set("plan", idlePlan(0)); state.set("obstacles", array());
            state.put("age_us", 0); return state;
        }
        long receiveAge = Math.max(0, nanoTime.getAsLong() - liveReceivedNs) / 1000;
        long ageUs = receiveAge + live.get("robot_us").longValue() - live.get("captured_us").longValue();
        boolean stale = ageUs >= LIVE_STALE_US;
        ObjectNode state = live.deepCopy(); state.put("mode", "live").put("age_us", ageUs);
        long producerNow = live.get("robot_us").longValue();
        state.put("estimated_robot_us", producerNow > MAX_ID - receiveAge ? MAX_ID : producerNow + receiveAge);
        String reason = stale ? "Telemetry stale: no planning or robot connection" : "Synthetic CLI producer; read-only display";
        if (!lastLiveRejection.isEmpty()) reason = "Rejected update: " + lastLiveRejection;
        state.set("connection", object().put("connected", !stale).put("stale", stale).put("reason", reason));
        ObjectNode plan = (ObjectNode)state.get("plan");
        boolean expired = plan.hasNonNull("valid_until_us") && state.get("estimated_robot_us").longValue() >= plan.get("valid_until_us").longValue();
        if (stale || expired || !lastLiveRejection.isEmpty()) {
            if ("SUCCESS".equals(plan.path("status").asText())) plan.put("status", "STALE_RESULT");
            plan.set("positions", array());
            plan.put("detail", stale ? "Stale telemetry; geometry hidden" : expired ? "Expired geometry hidden" : "Rejected update; geometry hidden");
        }
        if (!lastLiveRejection.isEmpty()) state.withArray("errors").add(lastLiveRejection);
        return state;
    }

    static ObjectNode normalizeFrame(JsonNode frame) {
        object(frame, "telemetry frame");
        if (!SCHEMA.equals(text(frame, "schema_version", 64)) || !"synthetic_mock".equals(text(frame, "source_kind", 64)))
            throw bad("Only explicitly synthetic_mock normalized telemetry is implemented");
        long robotUs = integer(frame, "robot_us", 0, MAX_ID);
        long capturedUs = optionalInteger(frame, "captured_us", robotUs, 0, robotUs);
        long epoch = integer(frame, "epoch", 0, MAX_ID), snapshot = integer(frame, "snapshot_id", 0, MAX_ID), map = integer(frame, "obstacle_map_version", 0, MAX_ID);
        ObjectNode state = base("live", "synthetic frame", true, false, robotUs, epoch, snapshot, map);
        state.put("captured_us", capturedUs).put("session_id", text(frame, "session_id", 128))
                .put("sequence", integer(frame, "sequence", 0, MAX_ID));
        state.set("field", field(required(frame, "field")));
        JsonNode robot = object(required(frame, "robot"), "robot");
        ObjectNode robotState = object(); robotState.set("pose", normalizedPose(required(robot, "pose")));
        robotState.set("velocity", normalizedVelocity(required(robot, "velocity"))); state.set("robot", robotState);
        ArrayNode obstacles = normalizeObstacles(frame.has("obstacles") ? frame.get("obstacles") : array());
        state.set("obstacles", obstacles);
        ObjectNode scene = object();
        JsonNode suppliedScenario = frame.get("scenario");
        if (suppliedScenario != null && !suppliedScenario.isNull()) {
            scene = normalizeScenario(merge(defaultScenario(), object(suppliedScenario, "scenario")));
            if (!scene.get("field").equals(state.get("field"))) throw bad("Telemetry scenario field differs from frame field");
            if (!scene.get("obstacles").equals(obstacles)) throw bad("Telemetry scenario obstacles differ from frame envelopes");
        }
        else {
            scene.set("bounds", defaultScenario().get("bounds")); scene.putNull("start"); scene.putNull("goal");
            scene.set("obstacles", obstacles.deepCopy()); scene.put("planning_supported", false);
        }
        state.set("scenario", scene); state.set("plan", normalizeRecordedPlan(frame.get("plan"), state));
        return state;
    }

    private static ObjectNode normalizeRecordedPlan(JsonNode node, ObjectNode frame) {
        if (node == null || node.isNull()) return idlePlan(0);
        object(node, "recorded plan");
        String status = text(node, "status", 32);
        if (!Set.of("SUCCESS", "NO_PATH", "TIMEOUT", "CANCELLED", "INVALID_INPUT", "STALE_RESULT", "IDLE", "PENDING").contains(status)) throw bad("Unknown recorded plan status");
        ObjectNode plan = idlePlan(status.equals("IDLE") ? optionalInteger(node, "generation", 0, 0, MAX_ID)
                : integer(node, "generation", 0, MAX_ID)).put("status", status);
        if (status.equals("IDLE")) return plan;
        plan.put("request_id", text(node, "request_id", 128)).put("task_id", text(node, "task_id", 128));
        for (String id : List.of("epoch", "snapshot_id", "obstacle_map_version")) {
            long value = integer(node, id, 0, MAX_ID);
            if (value != frame.get(id).longValue()) throw bad("Recorded plan " + id + " differs from frame");
            plan.put(id, value);
        }
        long issued = integer(node, "issued_us", 0, MAX_ID), expiry = integer(node, "valid_until_us", 0, MAX_ID);
        if (issued > frame.get("robot_us").longValue() || expiry <= issued) throw bad("Invalid recorded request time interval");
        plan.put("issued_us", issued).put("valid_until_us", expiry)
                .put("solver_duration_ns", integer(node, "solver_duration_ns", 0, MAX_ID))
                .put("backend_id", text(node, "backend_id", 128)).put("detail", optionalText(node, "detail", "Recorded synthetic geometry", 1024));
        if (node.hasNonNull("field") && !field(node.get("field")).equals(frame.get("field"))) throw bad("Recorded plan field mismatch");
        plan.set("field", frame.get("field").deepCopy());
        ArrayNode positions = node.has("positions") ? list(node, "positions", MAX_POINTS) : array();
        if (status.equals("SUCCESS")) {
            if (positions.size() < 2 || frame.get("robot_us").longValue() >= expiry) throw bad("Successful recorded geometry is absent or expired");
            for (JsonNode p : positions) plan.withArray("positions").add(normalizedPose(p));
        } else if (!positions.isEmpty()) throw bad("Non-success recorded plan cannot carry geometry");
        plan.put("point_count", positions.size()); return plan;
    }

    private static void validateProgress(ObjectNode previous, ObjectNode next, boolean replay) {
        boolean newSession = !previous.get("session_id").equals(next.get("session_id"));
        long oldEpoch = previous.get("epoch").longValue(), newEpoch = next.get("epoch").longValue();
        if (newEpoch < oldEpoch || (newSession && newEpoch <= oldEpoch)) throw bad("Producer restart requires a newer epoch");
        if (!newSession && next.get("sequence").longValue() <= previous.get("sequence").longValue()) throw bad("Non-increasing telemetry sequence");
        if (!newSession && next.get("robot_us").longValue() < previous.get("robot_us").longValue()) throw bad("Producer clock rollback requires a new session/epoch");
        if (newEpoch == oldEpoch) {
            if (!previous.get("field").equals(next.get("field"))) throw bad("Field change requires newer epoch");
            if (next.get("snapshot_id").longValue() < previous.get("snapshot_id").longValue()
                    || next.get("obstacle_map_version").longValue() < previous.get("obstacle_map_version").longValue()) throw bad("Snapshot/map version rolled backward");
        }
        JsonNode oldPlan = previous.get("plan"), newPlan = next.get("plan");
        if (!newSession && oldPlan.hasNonNull("request_id") && newPlan.hasNonNull("request_id")) {
            long oldGeneration = oldPlan.get("generation").longValue(), newGeneration = newPlan.get("generation").longValue();
            if (newGeneration < oldGeneration || (!oldPlan.get("request_id").equals(newPlan.get("request_id")) && newGeneration <= oldGeneration))
                throw bad("Recorded request generation cannot regress or reuse a retired generation");
        }
        if (!newSession && oldPlan.hasNonNull("request_id") && oldPlan.get("request_id").equals(newPlan.get("request_id"))) {
            for (String id : List.of("generation", "task_id", "epoch", "snapshot_id", "obstacle_map_version", "issued_us", "valid_until_us"))
                if (!oldPlan.get(id).equals(newPlan.get(id))) throw bad("Same request ID changed immutable metadata");
        }
    }
    private static boolean terminalFailure(String status) {
        return Set.of("CANCELLED", "STALE_RESULT", "NO_PATH", "TIMEOUT", "INVALID_INPUT").contains(status);
    }
    private static void validatePlanGeneration(long highest, String current, boolean revoked,
            long incoming, String request, String status) {
        if (incoming < highest || (incoming == highest && !request.equals(current)))
            throw bad("Retired request/generation cannot become current again");
        if (revoked && request.equals(current) && !terminalFailure(status))
            throw bad("Cancelled/terminal request cannot regain a path; submit a newer request");
    }

    static ObjectNode defaultScenario() {
        ObjectNode scenario = object();
        scenario.set("field", object().put("season", "offseason-synthetic").put("map_id", "dashboard-lab-8x4").put("geometry_revision", "1"));
        scenario.set("bounds", object().put("min_x_m", 0).put("min_y_m", 0).put("max_x_m", 8).put("max_y_m", 4));
        scenario.set("start", JsonSupport.pose(1, 2, 0).put("vx_mps", 0).put("vy_mps", 0).put("omega_radps", 0));
        scenario.set("goal", JsonSupport.pose(7, 2, 0).put("position_tolerance_m", .05).put("heading_tolerance_rad", .05).put("velocity_tolerance_mps", .05));
        scenario.set("footprint", object().put("length_m", .8).put("width_m", .8));
        scenario.set("constraints", object().put("max_speed_mps", 2).put("max_acceleration_mps2", 2).put("max_angular_speed_radps", 3));
        ArrayNode obstacles = array();
        obstacles.add(object().put("id", "static-lab").put("x_m", 4).put("y_m", 2).put("radius_m", .55).put("uncertainty_margin_m", .05).put("dynamic", false));
        obstacles.add(object().put("id", "dynamic-lab").put("x_m", 5.5).put("y_m", 3).put("radius_m", .25).put("uncertainty_margin_m", .05).put("dynamic", true));
        scenario.set("obstacles", obstacles); scenario.putNull("field_map"); scenario.put("planning_supported", true);
        scenario.set("warnings", array().add("SYNTHETIC generic 8 x 4 m lab; not verified FRC field geometry")); return scenario;
    }

    static ObjectNode normalizeScenario(ObjectNode input) {
        ObjectNode normalized = object(); normalized.set("field", field(required(input, "field")));
        JsonNode s = object(required(input, "start"), "start"), g = object(required(input, "goal"), "goal");
        normalized.set("start", normalizedPose(s).put("vx_mps", optionalNumber(s, "vx_mps", 0, -20, 20))
                .put("vy_mps", optionalNumber(s, "vy_mps", 0, -20, 20)).put("omega_radps", optionalNumber(s, "omega_radps", 0, -20, 20)));
        normalized.set("goal", normalizedPose(g).put("position_tolerance_m", optionalNumber(g, "position_tolerance_m", .05, 0, 2))
                .put("heading_tolerance_rad", optionalNumber(g, "heading_tolerance_rad", .05, 0, Math.PI))
                .put("velocity_tolerance_mps", optionalNumber(g, "velocity_tolerance_mps", .05, 0, 20)));
        JsonNode f = required(input, "footprint"), c = required(input, "constraints"), b = required(input, "bounds");
        normalized.set("footprint", object().put("length_m", number(f, "length_m", .01, 5)).put("width_m", number(f, "width_m", .01, 5)));
        normalized.set("constraints", object().put("max_speed_mps", number(c, "max_speed_mps", .01, 20))
                .put("max_acceleration_mps2", number(c, "max_acceleration_mps2", .01, 30)).put("max_angular_speed_radps", number(c, "max_angular_speed_radps", .01, 30)));
        double minX = number(b, "min_x_m", -100, 100), minY = number(b, "min_y_m", -100, 100),
                maxX = number(b, "max_x_m", -100, 100), maxY = number(b, "max_y_m", -100, 100);
        if (maxX <= minX || maxY <= minY) throw bad("Invalid rectangular bounds");
        normalized.set("bounds", object().put("min_x_m", minX).put("min_y_m", minY).put("max_x_m", maxX).put("max_y_m", maxY));
        normalized.set("obstacles", normalizeObstacles(required(input, "obstacles")));
        normalized.put("planning_supported", true); normalized.set("warnings", array().add("SYNTHETIC offline scenario; no physical qualification"));
        if (input.hasNonNull("field_map")) {
            FieldMapCompiler.Compiled compiled = FieldMapCompiler.compile(input.get("field_map"));
            normalized.set("field_map", input.get("field_map").deepCopy()); normalized.set("bounds", compiled.bounds());
            normalized.set("field", compiled.field()); normalized.set("obstacles", compiled.obstacles());
            normalized.put("planning_supported", compiled.supported()); normalized.withArray("warnings").addAll(compiled.warnings());
        } else normalized.putNull("field_map");
        return normalized;
    }

    static ArrayNode normalizeObstacles(JsonNode source) {
        if (!source.isArray() || source.size() > MAX_OBSTACLES) throw bad("Obstacles exceed bounded display/input capacity");
        ArrayNode result = array(); Set<String> ids = new HashSet<>();
        for (JsonNode o : source) {
            String id = text(o, "id", 128); if (!ids.add(id)) throw bad("Duplicate obstacle ID");
            ObjectNode circle = object().put("id", id).put("x_m", number(o, "x_m", -1000, 1000)).put("y_m", number(o, "y_m", -1000, 1000))
                    .put("radius_m", number(o, "radius_m", .001, 100)).put("uncertainty_margin_m", optionalNumber(o, "uncertainty_margin_m", 0, 0, 10))
                    .put("dynamic", optionalBoolean(o, "dynamic", false));
            if (o.hasNonNull("valid_until_us")) circle.put("valid_until_us", integer(o, "valid_until_us", 0, MAX_ID));
            result.add(circle);
        }
        return result;
    }

    private static ObjectNode merge(ObjectNode original, ObjectNode patch) {
        ObjectNode merged = original.deepCopy();
        patch.fields().forEachRemaining(entry -> {
            JsonNode previous = merged.get(entry.getKey()), next = entry.getValue();
            if (!entry.getKey().equals("field_map") && previous != null && previous.isObject() && next.isObject())
                merged.set(entry.getKey(), merge((ObjectNode)previous, (ObjectNode)next));
            else merged.set(entry.getKey(), next.deepCopy());
        }); return merged;
    }
    private static FieldIdentity fieldIdentity(JsonNode field) { return new FieldIdentity(field.get("season").textValue(), field.get("map_id").textValue(), field.get("geometry_revision").textValue()); }
    private static Pose2 pose(JsonNode p) { return new Pose2(new Vec2(p.get("x_m").doubleValue(), p.get("y_m").doubleValue()), p.get("heading_rad").doubleValue()); }
    private static ObjectNode robot(JsonNode p) {
        ObjectNode robot = object(); robot.set("pose", normalizedPose(p));
        robot.set("velocity", normalizedVelocity(p)); return robot;
    }
    private static ObjectNode idlePlan(long generation) {
        ObjectNode plan = object().put("status", "IDLE").put("generation", generation).put("solver_duration_ns", 0)
                .put("display_limit_exceeded", false).put("point_count", 0).put("geometry_kind", "geometric")
                .put("executable", false).put("detail", "Geometry display only; never robot authority or task completion");
        plan.putNull("request_id"); plan.putNull("task_id"); plan.set("positions", array()); return plan;
    }
    private static ObjectNode base(String mode, String reason, boolean connected, boolean stale, long robotUs, long epoch, long snapshot, long map) {
        ObjectNode state = object().put("schema_version", SCHEMA).put("mode", mode).put("source_kind", "synthetic_mock")
                .put("real_feed_implemented", false).put("robot_us", robotUs).put("epoch", epoch).put("snapshot_id", snapshot).put("obstacle_map_version", map);
        state.set("connection", object().put("connected", connected).put("stale", stale).put("reason", reason));
        state.set("errors", array()); return state;
    }
    private static List<ObjectNode> builtInReplay() {
        List<ObjectNode> frames = new ArrayList<>();
        for (int i = 0; i < 3; i++) {
            ObjectNode frame = base("replay", "synthetic recording", true, false, 1_000_000L + i * 200_000, 1, 1, 1);
            frame.put("session_id", "builtin-synthetic-replay").put("sequence", i).put("captured_us", frame.get("robot_us").longValue());
            frame.set("field", defaultScenario().get("field"));
            frame.set("robot", robot(JsonSupport.pose(1 + i, 1, 0).put("vx_mps", 1).put("vy_mps", 0).put("omega_radps", 0)));
            frame.set("obstacles", array());
            ObjectNode plan = idlePlan(1).put("status", "SUCCESS").put("request_id", "synthetic-recorded-path-1").put("task_id", "synthetic-replay-task")
                    .put("epoch", 1).put("snapshot_id", 1).put("obstacle_map_version", 1).put("issued_us", 1_000_000).put("valid_until_us", 5_000_000)
                    .put("backend_id", "synthetic-recording/not-a-solver").put("detail", "Synthetic authored recording; not a planner benchmark or hardware evidence");
            plan.withArray("positions").add(JsonSupport.pose(1, 1, 0)).add(JsonSupport.pose(7, 1, 0));
            frame.set("plan", plan); frames.add(normalizeFrame(frame));
        } return List.copyOf(frames);
    }
    @Override public void close() { planner.close(); }
    static final class UnsupportedGeometry extends IllegalArgumentException { private static final long serialVersionUID = 1L; }
    private static final class PlanRecord {
        final Request request; final long generation; final long budgetMs;
        volatile Result result; volatile String overrideStatus;
        PlanRecord(Request request, long generation, long budgetMs) { this.request = request; this.generation = generation; this.budgetMs = budgetMs; }
    }
}
