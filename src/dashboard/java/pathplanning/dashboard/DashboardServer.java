package pathplanning.dashboard;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpServer;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.URLDecoder;
import java.nio.charset.StandardCharsets;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import static pathplanning.dashboard.JsonSupport.*;

/** Loopback-only dashboard. No NetworkTables, robot connection, actuator or filesystem endpoint. */
public final class DashboardServer implements AutoCloseable {
    private static final int SMALL_BODY = 64 * 1024, REPLAY_BODY = 256 * 1024;
    private final HttpServer server;
    private final DashboardModel model;
    private final ThreadPoolExecutor httpWorkers;
    private final CountDownLatch stopped = new CountDownLatch(1);
    private final int port;
    private volatile boolean closed;

    public DashboardServer(int port) throws IOException {
        if (port < 1 || port > 65535) throw new IllegalArgumentException("port must be 1..65535");
        this.port = port;
        // Standalone desktop app JVM only. Verified against the selected JDK17 ServerConfig;
        // bound slow headers/bodies/responses in addition to the application executor queue.
        System.setProperty("jdk.httpserver.maxConnections", "32");
        System.setProperty("sun.net.httpserver.maxIdleConnections", "8");
        System.setProperty("sun.net.httpserver.maxReqHeaders", "64");
        System.setProperty("sun.net.httpserver.maxReqHeaderSize", "8192");
        System.setProperty("sun.net.httpserver.maxReqTime", "2");
        System.setProperty("sun.net.httpserver.maxRspTime", "5");
        System.setProperty("sun.net.httpserver.timerMillis", "200");
        System.setProperty("sun.net.httpserver.idleInterval", "10");
        System.setProperty("sun.net.httpserver.drainAmount", "0");
        server = HttpServer.create(new InetSocketAddress(InetAddress.getByAddress(new byte[]{127,0,0,1}), port), 16);
        model = new DashboardModel();
        AtomicInteger number = new AtomicInteger();
        httpWorkers = new ThreadPoolExecutor(2, 2, 0, TimeUnit.MILLISECONDS, new ArrayBlockingQueue<>(16), runnable -> {
            Thread thread = new Thread(runnable, "dashboard-http-" + number.incrementAndGet()); thread.setDaemon(true); return thread;
        }, new ThreadPoolExecutor.AbortPolicy());
        server.setExecutor(httpWorkers); server.createContext("/", this::handle);
    }

    public void start() { if (closed) throw new IllegalStateException("Server is closed"); server.start(); }
    public int port() { return port; }

    private void handle(HttpExchange exchange) throws IOException {
        try {
            validateHost(exchange);
            String method = exchange.getRequestMethod(), path = exchange.getRequestURI().getPath();
            if ("POST".equals(method)) validateMutation(exchange, path.equals("/api/live/ingest"));
            switch (path) {
                case "/api/state" -> {
                    requireMethod(method, "GET"); Map<String,String> query = query(exchange);
                    String mode = query.getOrDefault("mode", "sandbox");
                    if ("replay".equals(mode) && query.containsKey("index")) json(exchange, 200, model.replaySample(index(query.get("index"))));
                    else json(exchange, 200, model.state(mode));
                }
                case "/api/sandbox/scenario" -> { requireMethod(method, "POST"); json(exchange, 200, model.updateScenario(body(exchange, SMALL_BODY))); }
                case "/api/sandbox/plan" -> { requireMethod(method, "POST"); json(exchange, 202, model.plan(body(exchange, SMALL_BODY))); }
                case "/api/sandbox/cancel" -> { requireMethod(method, "POST"); body(exchange, SMALL_BODY); json(exchange, 200, model.cancel()); }
                case "/api/replay/load" -> { requireMethod(method, "POST"); json(exchange, 200, model.loadReplay(body(exchange, REPLAY_BODY))); }
                case "/api/replay/sample" -> { requireMethod(method, "GET"); json(exchange, 200, model.replaySample(index(query(exchange).getOrDefault("index", "0")))); }
                case "/api/live/ingest" -> { requireMethod(method, "POST"); json(exchange, 200, model.ingestLive(body(exchange, SMALL_BODY))); }
                case "/", "/index.html" -> { requireMethod(method, "GET"); resource(exchange, "index.html", "text/html; charset=utf-8"); }
                case "/app.js" -> { requireMethod(method, "GET"); resource(exchange, "app.js", "text/javascript; charset=utf-8"); }
                case "/style.css" -> { requireMethod(method, "GET"); resource(exchange, "style.css", "text/css; charset=utf-8"); }
                case "/field-import.js" -> { requireMethod(method, "GET"); resource(exchange, "field-import.js", "text/javascript; charset=utf-8"); }
                default -> json(exchange, 404, error("NOT_FOUND", "Unknown local endpoint"));
            }
        } catch (DashboardModel.UnsupportedGeometry unsupported) {
            json(exchange, 409, error("UNSUPPORTED_FIELD_GEOMETRY", "Import retained for rendering; planning requires a full rectangular boundary without holes"));
        } catch (HttpFailure rejected) {
            json(exchange, rejected.status, error(rejected.code, rejected.getMessage()));
        } catch (IllegalArgumentException rejected) {
            json(exchange, 400, error("INVALID_INPUT", safeDetail(rejected.getMessage())));
        } catch (com.fasterxml.jackson.core.JacksonException malformed) {
            json(exchange, 400, error("INVALID_JSON", "Malformed, duplicate-key, trailing-token or over-complex JSON"));
        } catch (RuntimeException failure) {
            json(exchange, 500, error("INTERNAL_ERROR", "Local dashboard request failed"));
        } finally { exchange.close(); }
    }

    private void validateHost(HttpExchange exchange) {
        if (!exchange.getRemoteAddress().getAddress().isLoopbackAddress()) throw new HttpFailure(403, "LOOPBACK_ONLY", "Only local clients are accepted");
        List<String> hosts = exchange.getRequestHeaders().get("Host");
        if (hosts == null || hosts.size() != 1) throw new HttpFailure(403, "INVALID_HOST", "Exactly one localhost Host is required");
        String host = hosts.get(0).toLowerCase(Locale.ROOT);
        if (!host.equals("127.0.0.1:" + port) && !host.equals("localhost:" + port))
            throw new HttpFailure(403, "INVALID_HOST", "Host must match this loopback server and port");
    }

    private void validateMutation(HttpExchange exchange, boolean liveIngest) {
        String origin = exchange.getRequestHeaders().getFirst("Origin");
        boolean fetchHeader = exchange.getRequestHeaders().keySet().stream().anyMatch(name -> name.toLowerCase(Locale.ROOT).startsWith("sec-fetch-"));
        if (liveIngest && (origin != null || fetchHeader))
            throw new HttpFailure(403, "CLI_PRODUCER_ONLY", "Live ingest rejects Origin and browser Sec-Fetch headers");
        if (origin != null && !origin.equals("http://127.0.0.1:" + port) && !origin.equals("http://localhost:" + port))
            throw new HttpFailure(403, "INVALID_ORIGIN", "Mutation Origin must exactly match the local dashboard");
        String site = exchange.getRequestHeaders().getFirst("Sec-Fetch-Site");
        if (site != null && !Set.of("same-origin", "none").contains(site))
            throw new HttpFailure(403, "CROSS_SITE_MUTATION", "Cross-site and same-site mutations are rejected");
        if (exchange.getRequestHeaders().get("Origin") != null && exchange.getRequestHeaders().get("Origin").size() != 1)
            throw new HttpFailure(403, "INVALID_ORIGIN", "Exactly one Origin is allowed");
    }

    private static JsonNode body(HttpExchange exchange, int limit) throws IOException {
        String contentType = exchange.getRequestHeaders().getFirst("Content-Type");
        if (contentType == null || !contentType.toLowerCase(Locale.ROOT).split(";", 2)[0].trim().equals("application/json"))
            throw new HttpFailure(415, "JSON_REQUIRED", "State mutations require application/json");
        List<String> lengths = exchange.getRequestHeaders().get("Content-Length");
        if (lengths != null) {
            if (lengths.size() != 1) throw new HttpFailure(400, "INVALID_LENGTH", "Invalid body length");
            long length;
            try { length = Long.parseLong(lengths.get(0)); }
            catch (NumberFormatException invalid) { throw new HttpFailure(400, "INVALID_LENGTH", "Invalid body length"); }
            if (length < 0) throw new HttpFailure(400, "INVALID_LENGTH", "Invalid body length");
            if (length > limit) throw new HttpFailure(413, "BODY_LIMIT", "JSON body exceeds " + limit + " bytes");
        }
        byte[] bytes = readBounded(exchange.getRequestBody(), limit);
        if (bytes.length == 0) return object();
        JsonNode parsed = MAPPER.readTree(bytes);
        return object(parsed, "JSON body");
    }

    private static byte[] readBounded(InputStream input, int limit) throws IOException {
        ByteArrayOutputStream bytes = new ByteArrayOutputStream(Math.min(limit, 4096));
        byte[] buffer = new byte[4096]; int read;
        while ((read = input.read(buffer)) >= 0) {
            if (read == 0) continue;
            if (bytes.size() > limit - read) throw new HttpFailure(413, "BODY_LIMIT", "Body exceeds bounded capacity");
            bytes.write(buffer, 0, read);
        } return bytes.toByteArray();
    }

    private static Map<String,String> query(HttpExchange exchange) {
        String raw = exchange.getRequestURI().getRawQuery(); Map<String,String> query = new HashMap<>();
        if (raw == null) return query;
        if (raw.length() > 1024) throw bad("Query exceeds bounded capacity");
        for (String pair : raw.split("&")) {
            String[] parts = pair.split("=", 2); String key = URLDecoder.decode(parts[0], StandardCharsets.UTF_8);
            String value = URLDecoder.decode(parts.length > 1 ? parts[1] : "", StandardCharsets.UTF_8);
            if (query.putIfAbsent(key, value) != null) throw bad("Duplicate query parameter");
        } return query;
    }
    private static int index(String value) {
        try { if (!value.matches("[0-9]{1,3}")) throw bad("Invalid replay index"); return Integer.parseInt(value); }
        catch (NumberFormatException invalid) { throw bad("Invalid replay index"); }
    }
    private static void requireMethod(String actual, String expected) {
        if (!actual.equals(expected)) throw new HttpFailure(405, "METHOD_NOT_ALLOWED", "Endpoint requires " + expected);
    }
    private static String safeDetail(String detail) { return detail == null ? "Invalid request" : detail.substring(0, Math.min(512, detail.length())); }

    private static void headers(HttpExchange exchange, String contentType) {
        exchange.getResponseHeaders().set("Content-Type", contentType);
        exchange.getResponseHeaders().set("Cache-Control", "no-store");
        exchange.getResponseHeaders().set("X-Content-Type-Options", "nosniff");
        exchange.getResponseHeaders().set("Referrer-Policy", "no-referrer");
        exchange.getResponseHeaders().set("Content-Security-Policy", "default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' blob: data:; connect-src 'self'; object-src 'none'; frame-ancestors 'none'; base-uri 'none'; form-action 'none'");
    }
    private static void json(HttpExchange exchange, int status, ObjectNode value) throws IOException {
        byte[] bytes = MAPPER.writeValueAsBytes(wire(value));
        if (bytes.length > 512 * 1024) { status = 500; bytes = MAPPER.writeValueAsBytes(error("RESPONSE_LIMIT", "Response exceeds bounded display capacity")); }
        headers(exchange, "application/json; charset=utf-8"); exchange.sendResponseHeaders(status, bytes.length); exchange.getResponseBody().write(bytes);
    }
    private static void resource(HttpExchange exchange, String name, String contentType) throws IOException {
        try (InputStream input = DashboardServer.class.getResourceAsStream("/dashboard/" + name)) {
            if (input == null) { json(exchange, 404, error("UI_RESOURCE_MISSING", "Dashboard resource is not on the app classpath")); return; }
            byte[] bytes = readBounded(input, 2 * 1024 * 1024); headers(exchange, contentType);
            exchange.sendResponseHeaders(200, bytes.length); exchange.getResponseBody().write(bytes);
        }
    }

    @Override public synchronized void close() {
        if (closed) return; closed = true;
        server.stop(0); model.close(); httpWorkers.shutdownNow(); stopped.countDown();
    }
    public static void main(String[] args) throws Exception {
        int port = 8086;
        if (args.length == 1 && args[0].equals("--help")) { System.out.println("DashboardServer [--port 8086] — loopback synthetic sandbox/replay/read-only telemetry"); return; }
        if (args.length != 0) {
            if (args.length != 2 || !args[0].equals("--port")) throw new IllegalArgumentException("Usage: DashboardServer [--port 8086]");
            port = Integer.parseInt(args[1]);
        }
        DashboardServer app = new DashboardServer(port);
        Runtime.getRuntime().addShutdownHook(new Thread(app::close, "dashboard-shutdown"));
        app.start(); System.out.println("Synthetic planner dashboard http://127.0.0.1:" + port + " — no robot connection; real feed not implemented");
        app.stopped.await();
    }
    private static final class HttpFailure extends RuntimeException {
        private static final long serialVersionUID = 1L;
        final int status; final String code;
        HttpFailure(int status, String code, String detail) { super(detail); this.status = status; this.code = code; }
    }
}
