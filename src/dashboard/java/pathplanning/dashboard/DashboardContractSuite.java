package pathplanning.dashboard;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import java.nio.file.Path;
import java.util.concurrent.TimeUnit;
import static pathplanning.dashboard.JsonSupport.*;

/** Small CPU-only app contracts. No HTTP socket, robot transport, GPU or benchmark. */
public final class DashboardContractSuite {
    private static int passed;
    private DashboardContractSuite() {}
    public static void main(String[] args) throws Exception {
        ObjectNode golden = object(MAPPER.readTree(Path.of("fixtures/field-map/synthetic-approved.json").toFile()), "golden");
        FieldMapCompiler.Compiled compiled = FieldMapCompiler.compile(golden);
        check(compiled.supported() && compiled.obstacles().size() == 1, "golden rectangular geometry");
        check(Math.abs(compiled.obstacles().get(0).get("radius_m").doubleValue()-Math.sqrt(2)) < 1e-12
                && compiled.obstacles().get(0).get("uncertainty_margin_m").doubleValue() == 0, "physical circle; no robot inflation");
        check(golden.get("obstacles").get(0).get("vertical_range_m").isNull(), "unknown height preserved");
        ObjectNode edit = golden.deepCopy(); ((ObjectNode)edit.get("map")).put("width_m", 9);
        rejects(() -> FieldMapCompiler.compile(edit), "approval digest rejects edits");
        mapReject(golden, map -> ((ObjectNode)map.get("calibration")).put("model", "homography"), "homography unsupported");
        mapReject(golden, map -> ((ArrayNode)map.get("calibration").get("image_to_field")).set(8, MAPPER.getNodeFactory().numberNode(2)), "affine bottom row");
        mapReject(golden, map -> ((ObjectNode)map.get("calibration")).put("independent_check_error_m", .1), "recomputed RMS");
        mapReject(golden, map -> ((ArrayNode)map.get("calibration").get("independent_check_points")).set(0, map.get("calibration").get("control_points").get(0).deepCopy()), "independent fitted pixel rejected");
        mapReject(golden, map -> ((ObjectNode)map.get("obstacles").get(0).get("review")).put("content_sha256", "copied"), "polygon review exact shape");
        mapReject(golden, map -> ((ObjectNode)map.get("image")).put("width_px", "400"), "field-map numeric integer type");
        mapReject(golden, map -> ((ObjectNode)map.get("obstacles").get(0)).put("uncertainty_margin_m", .1), "raw map forbids inflation");
        mapReject(golden, map -> ((ObjectNode)map.get("image")).put("file_name", "../field.png"), "image basename");
        mapReject(golden, map -> ((ArrayNode)map.get("obstacles").get(0).get("outer")).set(2, array().add(2).add(.5)), "self crossing/outside obstacle hole");
        ObjectNode nonrect = golden.deepCopy();
        ((ObjectNode)nonrect.get("boundary")).set("outer", array().add(array().add(0).add(0)).add(array().add(8).add(0))
                .add(array().add(7).add(4)).add(array().add(0).add(4)).add(array().add(0).add(0))); approve(nonrect);
        check(!FieldMapCompiler.compile(nonrect).supported(), "nonrect render import rejects planning");
        ObjectNode frame = SyntheticMockProducer.sample("test-session", 10, 0, 1_000_000, 1, .5);
        ((ObjectNode)frame.get("plan")).put("epoch", Long.MAX_VALUE-1).put("snapshot_id", Long.MAX_VALUE).put("obstacle_map_version", Long.MAX_VALUE-2);
        frame.put("epoch", Long.MAX_VALUE-1).put("snapshot_id", Long.MAX_VALUE).put("obstacle_map_version", Long.MAX_VALUE-2);
        JsonNode wire = wire(DashboardModel.normalizeFrame(frame));
        check(wire.get("snapshot_id").isTextual() && wire.get("snapshot_id").textValue().equals(Long.toString(Long.MAX_VALUE)), "full long wire preservation");
        check(integer(wire, "snapshot_id", 0, Long.MAX_VALUE) == Long.MAX_VALUE, "full long input round trip");
        try (DashboardModel model = new DashboardModel()) {
            check(!model.state("live").get("connection").get("connected").booleanValue(), "live disconnected by default");
            ObjectNode a = SyntheticMockProducer.sample("s", 10, 0, 1_000_000, 1, .5);
            model.ingestLive(a);
            rejects(() -> model.ingestLive(a), "duplicate sequence");
            check(model.state("live").get("plan").get("positions").isEmpty(), "rejection hides geometry");
            ObjectNode canceled = a.deepCopy(); canceled.put("sequence", 1); cancelRecorded(canceled); model.ingestLive(canceled);
            ObjectNode resurrected = a.deepCopy(); resurrected.put("sequence", 2);
            rejects(() -> model.ingestLive(resurrected), "cancel terminal latch");
            ObjectNode b = a.deepCopy(); b.put("sequence", 3); ((ObjectNode)b.get("plan")).put("generation", 2).put("request_id", "new-request-B"); model.ingestLive(b);
            ObjectNode idle = a.deepCopy(); idle.put("sequence", 4); idle.set("plan", object().put("status", "IDLE").put("generation", 2)); model.ingestLive(idle);
            ObjectNode retired = a.deepCopy(); retired.put("sequence", 5);
            rejects(() -> model.ingestLive(retired), "old request across IDLE");
            ObjectNode restarted = SyntheticMockProducer.sample("new-session", 11, 0, 1_000_000, 1, .5);
            restarted.set("plan", object().put("status", "IDLE")); model.ingestLive(restarted);
            ObjectNode restartedPlan = SyntheticMockProducer.sample("new-session", 11, 1, 1_000_001, 1, .5);
            check(model.ingestLive(restartedPlan).get("plan").get("status").asText().equals("SUCCESS"), "new-session IDLE resets generation");
            rejects(() -> model.ingestLive(a), "retired session");
            ObjectNode stale = restartedPlan.deepCopy(); stale.put("sequence", 2).put("captured_us", 0);
            rejects(() -> model.ingestLive(stale), "capture age rejects stale producer");
            ArrayNode frames = array(); ObjectNode replayA = SyntheticMockProducer.sample("replay", 1, 0, 1_000_000, 1, .5);
            ObjectNode replayB = replayA.deepCopy(); replayB.put("sequence", 1); ((ObjectNode)replayB.get("plan")).put("generation", 2).put("request_id", "B");
            ObjectNode replayIdle = replayA.deepCopy(); replayIdle.put("sequence", 2); replayIdle.set("plan", object().put("status", "IDLE"));
            ObjectNode replayOld = replayA.deepCopy(); replayOld.put("sequence", 3);
            frames.add(replayA).add(replayB).add(replayIdle).add(replayOld);
            ObjectNode replayDoc = object().put("schema_version", "frc-planner-dashboard-replay/1").put("source_kind", "synthetic_mock"); replayDoc.set("frames", frames);
            rejects(() -> model.loadReplay(replayDoc), "replay retired generation across IDLE");
            check(model.replaySample(0).get("robot").equals(model.replaySample(0).get("robot")), "deterministic replay sample");
            check(model.state("sandbox").get("plan").get("status").asText().equals("IDLE"), "live/replay never invoke sandbox planner");
            model.updateScenario(object().set("obstacles", array()));
            model.plan(object().put("budget_ms", 250).put("valid_for_ms", 30000));
            long deadline = System.nanoTime() + TimeUnit.SECONDS.toNanos(2); ObjectNode state;
            do { state = model.state("sandbox"); if (!state.get("plan").get("status").asText().equals("PENDING")) break; Thread.sleep(2); }
            while (System.nanoTime() < deadline);
            check(state.get("plan").get("status").asText().equals("SUCCESS") && state.get("plan").get("positions").size() >= 2, "actual backend sandbox geometry");
            model.cancel(); check(model.state("sandbox").get("plan").get("positions").isEmpty(), "cancel clears display proposal");
            model.updateScenario(object().set("field_map", nonrect));
            rejects(() -> model.plan(object()), "unsupported imported boundary fails closed");
        }
        System.out.println("DashboardContractSuite PASS " + passed + " bounded checks");
    }
    private static void cancelRecorded(ObjectNode frame) { ObjectNode plan = (ObjectNode)frame.get("plan"); plan.put("status", "CANCELLED"); plan.set("positions", array()); }
    private static void approve(ObjectNode map) { ((ObjectNode)map.get("approval")).put("content_sha256", FieldMapCompiler.contentDigest(map)); }
    private static void mapReject(ObjectNode golden, java.util.function.Consumer<ObjectNode> edit, String name) {
        ObjectNode changed = golden.deepCopy(); edit.accept(changed); approve(changed); rejects(() -> FieldMapCompiler.compile(changed), name);
    }
    private static void check(boolean condition, String name) { if (!condition) throw new AssertionError(name); passed++; }
    private static void rejects(Runnable task, String name) {
        try { task.run(); } catch (IllegalArgumentException expected) { passed++; return; }
        throw new AssertionError("Expected rejection: " + name);
    }
}
