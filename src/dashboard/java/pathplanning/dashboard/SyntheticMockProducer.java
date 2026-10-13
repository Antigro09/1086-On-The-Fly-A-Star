package pathplanning.dashboard;

import com.fasterxml.jackson.databind.node.ObjectNode;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.time.Duration;
import java.util.UUID;
import static pathplanning.dashboard.JsonSupport.*;

/** Bounded CLI-only synthetic producer; never reads a robot, NT server, file or external URL. */
public final class SyntheticMockProducer {
    private SyntheticMockProducer() {}
    public static void main(String[] args) throws Exception {
        int port = 8086, frames = 10, intervalMs = 200;
        for (int i = 0; i < args.length; i += 2) {
            if (i + 1 >= args.length) throw bad("Usage: SyntheticMockProducer [--port 8086] [--frames 10] [--interval-ms 200]");
            int value = Integer.parseInt(args[i+1]);
            switch (args[i]) { case "--port" -> port = value; case "--frames" -> frames = value; case "--interval-ms" -> intervalMs = value;
                default -> throw bad("Unknown producer option"); }
        }
        if (port < 1 || port > 65535 || frames < 1 || frames > 50 || intervalMs < 200 || intervalMs > 1000)
            throw bad("Producer bounds: port1..65535,frames1..50,interval200..1000ms");
        String session = "synthetic-cli-" + UUID.randomUUID();
        long epoch = System.currentTimeMillis(); // Opaque generation identity, never a robot/NT timestamp conversion.
        long began = System.nanoTime();
        HttpClient client = HttpClient.newBuilder().connectTimeout(Duration.ofSeconds(3)).build();
        for (int i = 0; i < frames; i++) {
            long nowUs = 1_000_000 + Math.max(0, System.nanoTime() - began)/1000;
            ObjectNode frame = sample(session, epoch, i, nowUs, 1 + .1*i, .1/(intervalMs/1000.0));
            HttpRequest request = HttpRequest.newBuilder(URI.create("http://127.0.0.1:" + port + "/api/live/ingest"))
                    .timeout(Duration.ofSeconds(3)).header("Content-Type", "application/json")
                    .POST(HttpRequest.BodyPublishers.ofString(MAPPER.writeValueAsString(wire(frame)))).build();
            HttpResponse<String> response = client.send(request, HttpResponse.BodyHandlers.ofString());
            if (response.statusCode() != 200) throw new IllegalStateException("Synthetic ingest rejected: HTTP" + response.statusCode() + " " + response.body());
            System.out.println("synthetic_mock sequence=" + i + " robot_us=" + nowUs + " accepted");
            if (i + 1 < frames) Thread.sleep(intervalMs);
        }
        System.out.println("Synthetic producer finished; dashboard live view becomes stale after 1 second. No real feed is implemented.");
    }
    static ObjectNode sample(String session, long epoch, long sequence, long nowUs, double x, double vx) {
        ObjectNode frame = object().put("schema_version", SCHEMA).put("source_kind", "synthetic_mock")
                .put("session_id", session).put("sequence", sequence).put("robot_us", nowUs).put("captured_us", nowUs)
                .put("epoch", epoch).put("snapshot_id", 1).put("obstacle_map_version", 1);
        frame.set("field", DashboardModel.defaultScenario().get("field"));
        ObjectNode robot = object(); robot.set("pose", JsonSupport.pose(x, 1, 0));
        robot.set("velocity", object().put("vx_mps", vx).put("vy_mps", 0).put("omega_radps", 0));
        frame.set("robot", robot); frame.set("obstacles", array());
        ObjectNode plan = object().put("status", "SUCCESS").put("generation", 1)
                .put("request_id", session + "-recorded-geometry").put("task_id", "synthetic-display-task")
                .put("epoch", epoch).put("snapshot_id", 1).put("obstacle_map_version", 1)
                .put("issued_us", 1_000_000).put("valid_until_us", 61_000_000).put("solver_duration_ns", 0)
                .put("backend_id", "synthetic-recording/not-a-solver")
                .put("detail", "Authored synthetic geometry and poses; no planner call, motion qualification or hardware evidence");
        plan.set("positions", array().add(JsonSupport.pose(1,1,0)).add(JsonSupport.pose(7,1,0)));
        frame.set("plan", plan); return frame;
    }
}
