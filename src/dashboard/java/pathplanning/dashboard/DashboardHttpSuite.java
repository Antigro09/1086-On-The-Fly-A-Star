package pathplanning.dashboard;

import java.io.InputStream;
import java.net.Socket;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.nio.charset.StandardCharsets;
import java.time.Duration;
import static pathplanning.dashboard.JsonSupport.*;

/** Local HTTP/security smoke. Run only in the authorized loopback test lane; exits its own server. */
public final class DashboardHttpSuite {
    private static int passed;
    private DashboardHttpSuite() {}
    public static void main(String[] args) throws Exception {
        int port = args.length == 0 ? 18086 : Integer.parseInt(args[0]);
        try (DashboardServer server = new DashboardServer(port)) {
            server.start(); HttpClient client = HttpClient.newBuilder().connectTimeout(Duration.ofSeconds(3)).build();
            String base = "http://127.0.0.1:" + port;
            check(send(client, request(base+"/api/state?mode=live", null)).statusCode() == 200, "loopback state");
            check(send(client, request(base+"/api/sandbox/plan", "{}").header("Origin", "https://foreign.example")).statusCode() == 403, "foreign Origin");
            check(send(client, request(base+"/api/sandbox/plan", "{}").header("Sec-Fetch-Site", "same-site")).statusCode() == 403, "same-site mutation");
            String frame = MAPPER.writeValueAsString(wire(SyntheticMockProducer.sample("http", 1, 0, 1_000_000, 1, .5)));
            check(send(client, request(base+"/api/live/ingest", frame).header("Origin", base)).statusCode() == 403, "browser live ingest");
            check(send(client, request(base+"/api/live/ingest", frame).header("Sec-Fetch-Mode", "cors")).statusCode() == 403, "fetch live ingest");
            check(send(client, request(base+"/api/live/ingest", frame)).statusCode() == 200, "CLI synthetic ingest");
            check(send(client, request(base+"/api/sandbox/plan", "{\"budget_ms\":250,\"budget_ms\":1}")).statusCode() == 400, "duplicate keys");
            check(send(client, request(base+"/api/sandbox/plan", "{} {}" )).statusCode() == 400, "trailing JSON");
            check(send(client, request(base+"/api/sandbox/plan", " ".repeat(65537))).statusCode() == 413, "raw body limit");
            check(send(client, request(base+"/api/sandbox/cancel", "{}" )).statusCode() == 200, "cancel route");
            try (Socket socket = new Socket("127.0.0.1", port)) {
                socket.setSoTimeout(4000);
                socket.getOutputStream().write(("GET /api/state HTTP/1.1\r\nHost:foreign.example:"+port+"\r\nConnection:close\r\n\r\n").getBytes(StandardCharsets.US_ASCII));
                String first = new String(socket.getInputStream().readNBytes(40), StandardCharsets.US_ASCII);
                check(first.contains("403"), "DNS-rebinding Host");
            }
            try (Socket socket = new Socket("127.0.0.1", port)) {
                socket.setSoTimeout(4500); long began = System.nanoTime();
                socket.getOutputStream().write(("POST /api/sandbox/plan HTTP/1.1\r\nHost:127.0.0.1:"+port+"\r\nContent-Type:application/json\r\nContent-Length:10\r\n\r\n{").getBytes(StandardCharsets.US_ASCII));
                int read = socket.getInputStream().read();
                check(read == -1 && (System.nanoTime()-began) < 4_000_000_000L, "slow body deadline");
            }
        }
        System.out.println("DashboardHttpSuite PASS " + passed + " loopback checks");
    }
    private static HttpRequest.Builder request(String uri, String body) {
        HttpRequest.Builder request = HttpRequest.newBuilder(URI.create(uri)).timeout(Duration.ofSeconds(4));
        return body == null ? request.GET() : request.header("Content-Type", "application/json").POST(HttpRequest.BodyPublishers.ofString(body));
    }
    private static HttpResponse<String> send(HttpClient client, HttpRequest.Builder request) throws Exception { return client.send(request.build(), HttpResponse.BodyHandlers.ofString()); }
    private static void check(boolean condition, String name) { if (!condition) throw new AssertionError(name); passed++; }
}
