package pathplanning.dashboard;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.core.StreamReadConstraints;
import com.fasterxml.jackson.core.StreamReadFeature;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import java.util.Set;

/** Strict bounded JSON at the app boundary; this dependency does not enter the planner core. */
final class JsonSupport {
    static final ObjectMapper MAPPER = new ObjectMapper(JsonFactory.builder()
            .streamReadConstraints(StreamReadConstraints.builder().maxNestingDepth(32)
                    .maxNumberLength(64).maxStringLength(65536).build())
            .enable(StreamReadFeature.STRICT_DUPLICATE_DETECTION).build())
            .enable(DeserializationFeature.FAIL_ON_TRAILING_TOKENS);
    static final String SCHEMA = "frc-planner-dashboard/1";
    static final int MAX_POINTS = 512;
    static final int MAX_OBSTACLES = 128;
    private static final Set<String> LONG_FIELDS = Set.of("robot_us", "captured_us", "estimated_robot_us", "age_us",
            "epoch", "snapshot_id", "obstacle_map_version", "sequence", "generation", "issued_us", "valid_until_us", "solver_duration_ns");
    private JsonSupport() {}

    static ObjectNode object() { return MAPPER.createObjectNode(); }
    static ArrayNode array() { return MAPPER.createArrayNode(); }
    static ObjectNode object(JsonNode node, String name) {
        if (node == null || !node.isObject()) throw bad(name + " must be an object");
        return (ObjectNode) node;
    }
    static JsonNode required(JsonNode node, String name) {
        JsonNode value = node.get(name);
        if (value == null || value.isNull()) throw bad("Missing " + name);
        return value;
    }
    static String text(JsonNode node, String name, int max) {
        JsonNode value = required(node, name);
        if (!value.isTextual() || value.textValue().isBlank() || value.textValue().length() > max)
            throw bad("Invalid " + name);
        String text = value.textValue();
        for (int i = 0; i < text.length(); i++) {
            char c = text.charAt(i);
            if (Character.isHighSurrogate(c)) {
                if (i + 1 >= text.length() || !Character.isLowSurrogate(text.charAt(i+1))) throw bad("Unpaired Unicode surrogate in " + name);
            } else if (Character.isLowSurrogate(c) && (i == 0 || !Character.isHighSurrogate(text.charAt(i-1)))) throw bad("Unpaired Unicode surrogate in " + name);
        }
        return text;
    }
    static String optionalText(JsonNode node, String name, String fallback, int max) {
        return !node.hasNonNull(name) ? fallback : text(node, name, max);
    }
    static long integer(JsonNode node, String name, long min, long max) {
        JsonNode value = required(node, name);
        long n;
        if (value.isTextual() && value.textValue().matches("0|[1-9][0-9]{0,18}")) {
            try { n = Long.parseLong(value.textValue()); } catch (NumberFormatException tooLarge) { throw bad("Out of range " + name); }
        } else if (value.isIntegralNumber() && value.canConvertToLong()) n = value.longValue();
        else throw bad(name + " must be an exact nonnegative integer or decimal string");
        if (n < min || n > max) throw bad("Out of range " + name);
        return n;
    }
    /** Decimal strings preserve the SPI's full nonnegative signed-long identities/times in JS. */
    static JsonNode wire(JsonNode value) {
        if (value.isArray()) { ArrayNode out = array(); for (JsonNode item : value) out.add(wire(item)); return out; }
        if (value.isObject()) {
            ObjectNode out = object();
            value.fields().forEachRemaining(entry -> {
                JsonNode child = entry.getValue();
                if (LONG_FIELDS.contains(entry.getKey()) && child.isIntegralNumber()) out.put(entry.getKey(), child.bigIntegerValue().toString());
                else out.set(entry.getKey(), wire(child));
            }); return out;
        }
        return value.deepCopy();
    }
    static long optionalInteger(JsonNode node, String name, long fallback, long min, long max) {
        return !node.hasNonNull(name) ? fallback : integer(node, name, min, max);
    }
    static double number(JsonNode node, String name, double min, double max) {
        JsonNode value = required(node, name);
        if (!value.isNumber()) throw bad(name + " must be numeric");
        double n = value.doubleValue();
        if (!Double.isFinite(n) || n < min || n > max) throw bad("Out of range " + name);
        return n;
    }
    static double optionalNumber(JsonNode node, String name, double fallback, double min, double max) {
        return !node.hasNonNull(name) ? fallback : number(node, name, min, max);
    }
    static boolean optionalBoolean(JsonNode node, String name, boolean fallback) {
        if (!node.has(name)) return fallback;
        if (!node.get(name).isBoolean()) throw bad(name + " must be boolean");
        return node.get(name).booleanValue();
    }
    static ArrayNode list(JsonNode node, String name, int max) {
        JsonNode value = required(node, name);
        if (!value.isArray() || value.size() > max) throw bad("Invalid/bounded list " + name);
        return (ArrayNode) value;
    }
    static IllegalArgumentException bad(String message) { return new IllegalArgumentException(message); }
    static ObjectNode pose(double x, double y, double heading) {
        return object().put("x_m", x).put("y_m", y).put("heading_rad", heading);
    }
    static ObjectNode normalizedPose(JsonNode node) {
        return pose(number(node, "x_m", -1000, 1000), number(node, "y_m", -1000, 1000),
                number(node, "heading_rad", -100000, 100000));
    }
    static ObjectNode normalizedVelocity(JsonNode node) {
        return object().put("vx_mps", number(node, "vx_mps", -100, 100))
                .put("vy_mps", number(node, "vy_mps", -100, 100))
                .put("omega_radps", number(node, "omega_radps", -100, 100));
    }
    static ObjectNode field(JsonNode node) {
        return object().put("season", text(node, "season", 1024)).put("map_id", text(node, "map_id", 128))
                .put("geometry_revision", text(node, "geometry_revision", 128));
    }
    static ObjectNode error(String code, String detail) {
        return object().put("schema_version", SCHEMA).put("error", code).put("detail", detail);
    }
}
