package pathplanning.dashboard;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.util.ArrayList;
import java.util.HexFormat;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import static pathplanning.dashboard.JsonSupport.*;

/** Import boundary: conservative physical circles only; robot inflation belongs to AStar backend. */
final class FieldMapCompiler {
    record Compiled(ObjectNode bounds, ObjectNode field, ArrayNode obstacles, boolean supported,
                    ArrayNode warnings) {}
    private FieldMapCompiler() {}

    static Compiled compile(JsonNode document) {
        ObjectNode doc = object(document, "field_map");
        keys(doc, Set.of("schema_version", "map", "image", "source", "calibration", "boundary", "obstacles", "approval"));
        if (!"frc-field-map/1".equals(text(doc, "schema_version", 64))) throw bad("Unsupported field-map schema");
        preflightVertices(doc);
        JsonNode map = object(required(doc, "map"), "map");
        keys(map, Set.of("id", "revision", "season", "variant", "frame", "units", "width_m", "height_m"));
        if (!"wpilib_nwu".equals(text(map, "frame", 64)) || !"m".equals(text(map, "units", 8)))
            throw bad("Field map must use metric wpilib_nwu coordinates");
        long revision = schemaInteger(map, "revision", 1, 9_007_199_254_740_991L);
        text(map, "id", 128); text(map, "variant", 128); nullableText(map, "season", 1024);
        double width = number(map, "width_m", Double.MIN_VALUE, 100), height = number(map, "height_m", Double.MIN_VALUE, 100);
        keys(required(doc, "approval"), Set.of("state", "reviewed_revision", "content_sha256"));
        reviewed(required(doc, "approval"), revision);
        String expected = text(doc.get("approval"), "content_sha256", 64);
        if (!expected.matches("[0-9a-f]{64}") || !expected.equals(contentDigest(doc)))
            throw bad("Field-map approval content digest does not match its geometry/provenance");
        JsonNode image = object(required(doc, "image"), "image");
        keys(image, Set.of("file_name", "sha256", "width_px", "height_px", "mime_type", "attribution", "license"));
        if (!text(image, "sha256", 64).matches("[0-9a-f]{64}")) throw bad("Invalid image SHA256 metadata");
        String fileName = text(image, "file_name", 128);
        if (fileName.startsWith(".") || !fileName.matches("(?i)[^/\\\\:]+\\.(png|jpe?g|webp)")) throw bad("Image name must be a local static-raster basename");
        if (!Set.of("image/png", "image/jpeg", "image/webp").contains(text(image, "mime_type", 64))) throw bad("Unsupported raster MIME type");
        long imageWidth = schemaInteger(image, "width_px", 1, 8192), imageHeight = schemaInteger(image, "height_px", 1, 8192);
        if (imageWidth * imageHeight > 16_000_000) throw bad("Raster metadata exceeds 16 megapixels");
        text(image, "attribution", 1024); nullableText(image, "license", 1024);
        JsonNode source = required(doc, "source"); keys(source, Set.of("kind", "label", "uri"));
        if (!Set.of("synthetic", "user_image", "official_geometry").contains(text(source, "kind", 64))) throw bad("Invalid map source kind");
        text(source, "label", 1024); nullableText(source, "uri", 1024);
        JsonNode calibration = object(required(doc, "calibration"), "calibration");
        keys(calibration, Set.of("model", "image_to_field", "control_points", "distortion", "fit_error_m", "independent_check_error_m", "independent_check_points"));
        String calibrationModel = text(calibration, "model", 32);
        if (!calibrationModel.equals("affine")) throw bad("Only affine calibration is supported by frc-field-map/1");
        ArrayNode transform = list(calibration, "image_to_field", 9);
        if (transform.size() != 9) throw bad("Calibration transform must have nine coefficients");
        double[] matrix = new double[9]; for (int i = 0; i < 9; i++) matrix[i] = finitePointNumber(transform.get(i));
        double determinant = matrix[0]*matrix[4] - matrix[1]*matrix[3];
        if (matrix[6] != 0 || matrix[7] != 0 || matrix[8] != 1 || !Double.isFinite(determinant) || Math.abs(determinant) < 1e-12)
            throw bad("Affine transform must have [0,0,1] bottom row and invertible finite 2x2 matrix");
        if (!Set.of("uncorrected", "corrected", "not_applicable").contains(text(calibration, "distortion", 32))) throw bad("Invalid distortion metadata");
        ArrayNode controls = list(calibration, "control_points", 32), checks = list(calibration, "independent_check_points", 32);
        if (controls.size() < 2 || checks.isEmpty()) throw bad("Approved calibration requires fit points and independent check points");
        Set<String> controlPixels = new HashSet<>();
        double fitRms = residual(controls, matrix, imageWidth, imageHeight, width, height, controlPixels, null);
        double checkRms = residual(checks, matrix, imageWidth, imageHeight, width, height, new HashSet<>(), controlPixels);
        if (Math.abs(number(calibration, "fit_error_m", 0, 1000) - fitRms) > 1e-8
                || Math.abs(number(calibration, "independent_check_error_m", 0, 1000) - checkRms) > 1e-8)
            throw bad("Declared calibration RMS residual differs from independent recomputation");
        JsonNode boundary = object(required(doc, "boundary"), "boundary");
        keys(boundary, Set.of("outer", "holes", "review", "provenance")); provenance(required(boundary, "provenance"));
        keys(required(boundary, "review"), Set.of("state", "reviewed_revision"));
        reviewed(required(boundary, "review"), revision);
        List<double[]> outer = ring(required(boundary, "outer"), true, width, height);
        ArrayNode boundaryHoles = list(boundary, "holes", 16);
        List<List<double[]>> boundaryHoleRings = holes(boundaryHoles, outer, width, height);
        int totalVertices = outer.size() - 1; for (List<double[]> hole : boundaryHoleRings) totalVertices += hole.size() - 1;
        if (totalVertices > 4096) throw bad("Field map exceeds 4096 total vertices");
        boolean supported = boundaryHoles.isEmpty() && rectangle(outer, width, height);
        ArrayNode warnings = array();
        if (!supported) warnings.add("UNSUPPORTED_FIELD_GEOMETRY: planner requires a full axis-aligned rectangle without boundary holes; import retained for rendering");
        ArrayNode envelopes = array(); Set<String> ids = new HashSet<>();
        for (JsonNode polygon : list(doc, "obstacles", 64)) {
            keys(polygon, Set.of("id", "outer", "holes", "review", "provenance", "vertical_range_m"));
            String id = text(polygon, "id", 128);
            if (!ids.add(id)) throw bad("Duplicate physical obstacle ID");
            keys(required(polygon, "review"), Set.of("state", "reviewed_revision"));
            reviewed(required(polygon, "review"), revision);
            provenance(required(polygon, "provenance"));
            JsonNode vertical = polygon.get("vertical_range_m");
            if (vertical == null) throw bad("Missing vertical range; unknown height must be null");
            if (!vertical.isNull()) {
                keys(vertical, Set.of("min", "max"));
                if (number(vertical, "min", -1000, 1000) >= number(vertical, "max", -1000, 1000)) throw bad("Invalid known vertical range");
            }
            List<double[]> vertices = ring(required(polygon, "outer"), true, width, height);
            ArrayNode holes = list(polygon, "holes", 16);
            List<List<double[]>> obstacleHoles = holes(holes, vertices, width, height);
            requireContained(vertices, outer, false);
            for (List<double[]> excluded : boundaryHoleRings)
                if (ringsIntersect(vertices, excluded, false) || inside(vertices.get(0), excluded, true) || inside(excluded.get(0), vertices, true))
                    throw bad("Physical obstacle intersects an excluded boundary hole");
            int count = vertices.size() - 1; for (List<double[]> hole : obstacleHoles) count += hole.size() - 1;
            // Total schema capacity, independently of the HTTP body-byte bound.
            totalVertices += count;
            if (totalVertices > 4096) throw bad("Field map exceeds 4096 total vertices");
            if (!holes.isEmpty()) warnings.add("Physical obstacle " + id + ": holes filled conservatively by circumscribed circle");
            double minX = Double.POSITIVE_INFINITY, minY = minX, maxX = Double.NEGATIVE_INFINITY, maxY = maxX;
            for (double[] p : vertices) { minX = Math.min(minX, p[0]); minY = Math.min(minY, p[1]); maxX = Math.max(maxX, p[0]); maxY = Math.max(maxY, p[1]); }
            double cx = (minX + maxX) / 2, cy = (minY + maxY) / 2, radius = 0;
            for (double[] p : vertices) radius = Math.max(radius, Math.hypot(p[0] - cx, p[1] - cy));
            envelopes.add(object().put("id", id).put("x_m", cx).put("y_m", cy).put("radius_m", radius)
                    .put("uncertainty_margin_m", 0)
                    .put("dynamic", false).put("envelope_source", "physical_polygon_circumscribed_circle"));
        }
        warnings.add("Physical polygons become conservative circles; robot footprint/clearance inflation is owned only by AStarPlannerBackend");
        return new Compiled(object().put("min_x_m", 0).put("min_y_m", 0).put("max_x_m", width).put("max_y_m", height),
                object().put("season", optionalText(map, "season", "offseason-unspecified", 1024))
                        .put("map_id", text(map, "id", 128)).put("geometry_revision", Long.toString(revision)),
                envelopes, supported, warnings);
    }

    static String contentDigest(ObjectNode document) {
        ObjectNode content = document.deepCopy(); content.remove("approval");
        try { return HexFormat.of().formatHex(MessageDigest.getInstance("SHA-256").digest(canonical(content).getBytes(StandardCharsets.UTF_8))); }
        catch (java.security.NoSuchAlgorithmException impossible) { throw new IllegalStateException(impossible); }
    }

    /** Same tagged-f64, codepoint-sorted canonical JSON as the image importer's golden fixture. */
    static String canonical(JsonNode value) {
        if (value.isNumber()) {
            double n = value.doubleValue();
            if (!Double.isFinite(n)) throw bad("Nonfinite canonical number");
            if (n == 0) n = 0;
            return quote("f64:" + String.format(java.util.Locale.ROOT, "%016x", Double.doubleToLongBits(n)));
        }
        if (value.isTextual()) return quote(value.textValue());
        if (value.isNull() || value.isBoolean()) return value.toString();
        StringBuilder result = new StringBuilder();
        if (value.isArray()) {
            result.append('['); boolean first = true;
            for (JsonNode item : value) { if (!first) result.append(','); result.append(canonical(item)); first = false; }
            return result.append(']').toString();
        }
        List<String> keys = new ArrayList<>(); value.fieldNames().forEachRemaining(keys::add);
        keys.sort(FieldMapCompiler::compareCodePoints); result.append('{'); boolean first = true;
        for (String key : keys) { if (!first) result.append(','); result.append(quote(key)).append(':').append(canonical(value.get(key))); first = false; }
        return result.append('}').toString();
    }
    private static int compareCodePoints(String a, String b) {
        int ai = 0, bi = 0;
        while (ai < a.length() && bi < b.length()) {
            int ac = a.codePointAt(ai), bc = b.codePointAt(bi);
            if (ac != bc) return Integer.compare(ac, bc);
            ai += Character.charCount(ac); bi += Character.charCount(bc);
        }
        return Integer.compare(a.length() - ai, b.length() - bi);
    }
    private static String quote(String text) {
        // Python/JS importer canonical escaping, including lowercase control-code hex.
        StringBuilder s = new StringBuilder("\"");
        for (int i = 0; i < text.length(); i++) {
            char c = text.charAt(i);
            if (Character.isHighSurrogate(c)) {
                if (i + 1 >= text.length() || !Character.isLowSurrogate(text.charAt(i + 1))) throw bad("Unpaired Unicode surrogate");
            } else if (Character.isLowSurrogate(c) && (i == 0 || !Character.isHighSurrogate(text.charAt(i - 1)))) throw bad("Unpaired Unicode surrogate");
            switch (c) {
                case '"' -> s.append("\\\""); case '\\' -> s.append("\\\\");
                case '\b' -> s.append("\\b"); case '\f' -> s.append("\\f");
                case '\n' -> s.append("\\n"); case '\r' -> s.append("\\r"); case '\t' -> s.append("\\t");
                default -> { if (c < 32) s.append(String.format(java.util.Locale.ROOT, "\\u%04x", (int)c)); else s.append(c); }
            }
        }
        return s.append('"').toString();
    }
    private static void reviewed(JsonNode node, long revision) {
        object(node, "review");
        // Whole approval contains one additional digest field; polygon review contains exactly two.
        Set<String> allowed = node.has("content_sha256") ? Set.of("state", "reviewed_revision", "content_sha256") : Set.of("state", "reviewed_revision");
        keys(node, allowed);
        if (!"approved".equals(text(node, "state", 32)) || schemaInteger(node, "reviewed_revision", 1, 9_007_199_254_740_991L) != revision)
            throw bad("Field-map geometry must be explicitly approved at its current revision");
    }
    private static long schemaInteger(JsonNode node, String name, long min, long max) {
        JsonNode value = required(node, name);
        if (!value.isNumber()) throw bad("frc-field-map/1 " + name + " must be a JSON numeric integer");
        double n = value.doubleValue();
        if (!Double.isFinite(n) || n != Math.rint(n) || n < min || n > max) throw bad("Out of range numeric integer " + name);
        return (long)n;
    }
    private static void preflightVertices(JsonNode doc) {
        JsonNode boundary = required(doc, "boundary"); int count = ringCount(required(boundary, "outer"));
        for (JsonNode ring : list(boundary, "holes", 16)) count += ringCount(ring);
        for (JsonNode obstacle : list(doc, "obstacles", 64)) {
            count += ringCount(required(obstacle, "outer"));
            for (JsonNode ring : list(obstacle, "holes", 16)) count += ringCount(ring);
            if (count > 4096) throw bad("Field map exceeds 4096 total vertices");
        }
        if (count > 4096) throw bad("Field map exceeds 4096 total vertices");
    }
    private static int ringCount(JsonNode ring) {
        if (!ring.isArray() || ring.size() < 4 || ring.size() > 257) throw bad("Invalid bounded polygon ring");
        return ring.size()-1;
    }
    private static void keys(JsonNode node, Set<String> allowed) {
        object(node, "field-map object");
        node.fieldNames().forEachRemaining(name -> { if (!allowed.contains(name)) throw bad("Unknown field-map property " + name); });
    }
    private static void nullableText(JsonNode node, String name, int max) {
        if (!node.has(name)) throw bad("Missing " + name + "; unknown values must be null");
        JsonNode value = node.get(name);
        if (!value.isNull() && (!value.isTextual() || value.textValue().length() > max)) throw bad("Invalid nullable " + name);
    }
    private static void provenance(JsonNode node) {
        keys(node, Set.of("kind", "label", "uri"));
        if (!Set.of("suggested", "manual", "verified_source").contains(text(node, "kind", 64))) throw bad("Invalid polygon provenance");
        text(node, "label", 1024); nullableText(node, "uri", 1024);
    }
    private static double residual(ArrayNode points, double[] m, long imageWidth, long imageHeight,
            double width, double height, Set<String> seen, Set<String> fitted) {
        double sum = 0;
        for (JsonNode point : points) {
            keys(point, Set.of("pixel", "field_m"));
            ArrayNode pixel = list(point, "pixel", 2), field = list(point, "field_m", 2);
            if (pixel.size() != 2 || field.size() != 2) throw bad("Calibration points need two pixel/metric coordinates");
            double u = finitePointNumber(pixel.get(0)), v = finitePointNumber(pixel.get(1)),
                    x = finitePointNumber(field.get(0)), y = finitePointNumber(field.get(1));
            if (u < 0 || u > imageWidth || v < 0 || v > imageHeight || x < 0 || x > width || y < 0 || y > height)
                throw bad("Calibration point outside raster/field bounds");
            String identity = (u == 0 ? "0" : Double.toString(u)) + "/" + (v == 0 ? "0" : Double.toString(v));
            if (!seen.add(identity) || (fitted != null && fitted.contains(identity))) throw bad("Calibration checks must use distinct unfitted pixels");
            double dx = m[0]*u + m[1]*v + m[2] - x, dy = m[3]*u + m[4]*v + m[5] - y;
            sum += dx*dx + dy*dy;
        }
        double result = Math.sqrt(sum / points.size());
        if (!Double.isFinite(result)) throw bad("Nonfinite projected calibration residual");
        return result;
    }
    private static double finitePointNumber(JsonNode value) {
        if (!value.isNumber() || !Double.isFinite(value.doubleValue())) throw bad("Nonfinite/non-numeric map coordinate");
        return value.doubleValue();
    }
    private static List<double[]> ring(JsonNode value, boolean ccw, double width, double height) {
        if (!value.isArray() || value.size() < 4 || value.size() > 257) throw bad("Invalid bounded polygon ring");
        List<double[]> points = new ArrayList<>();
        for (JsonNode point : value) {
            if (!point.isArray() || point.size() != 2) throw bad("Map vertex must have two coordinates");
            double x = finitePointNumber(point.get(0)), y = finitePointNumber(point.get(1));
            if (x < 0 || x > width || y < 0 || y > height) throw bad("Map polygon outside declared dimensions");
            points.add(new double[]{x, y});
        }
        double[] first = points.get(0), last = points.get(points.size() - 1);
        if (first[0] != last[0] || first[1] != last[1]) throw bad("Map ring must be closed");
        double area = 0;
        for (int i = 1; i < points.size(); i++) {
            double[] a = points.get(i - 1), b = points.get(i);
            if (a[0] == b[0] && a[1] == b[1]) throw bad("Degenerate polygon edge");
            area += a[0]*b[1] - b[0]*a[1];
        }
        if (!Double.isFinite(area) || Math.abs(area) < 2e-8 || (area > 0) != ccw) throw bad("Map ring orientation/area invalid");
        int edges = points.size() - 1;
        for (int i = 0; i < edges; i++) {
            double[] a = points.get(i), b = points.get(i+1), c = points.get((i+2) % edges);
            if (Math.abs(cross(a,b,c)) <= 1e-10 && (b[0]-a[0])*(c[0]-b[0])+(b[1]-a[1])*(c[1]-b[1]) < 0)
                throw bad("Adjacent polygon edges overlap");
            for (int j = i+1; j < edges; j++) {
                if (j == i+1 || (i == 0 && j == edges-1)) continue;
                if (intersects(a,b,points.get(j),points.get(j+1),false)) throw bad("Polygon self-intersection");
            }
        }
        return points;
    }
    private static List<List<double[]>> holes(ArrayNode source, List<double[]> outer, double width, double height) {
        List<List<double[]>> holes = new ArrayList<>();
        for (JsonNode value : source) {
            List<double[]> next = ring(value, false, width, height); requireContained(next, outer, true);
            for (List<double[]> old : holes) if (ringsIntersect(next, old, false) || inside(next.get(0), old, true) || inside(old.get(0), next, true))
                throw bad("Polygon holes overlap/intersect");
            holes.add(next);
        } return holes;
    }
    private static void requireContained(List<double[]> inner, List<double[]> outer, boolean strict) {
        for (int i = 0; i < inner.size()-1; i++) {
            double[] a = inner.get(i), b = inner.get(i+1);
            if (!inside(a, outer, !strict) || !inside(new double[]{(a[0]+b[0])/2,(a[1]+b[1])/2}, outer, !strict))
                throw bad("Polygon/hole outside its containing boundary");
        }
        if (ringsIntersect(inner, outer, !strict)) throw bad("Polygon/hole crosses containing boundary");
    }
    private static boolean inside(double[] p, List<double[]> ring, boolean allowBoundary) {
        boolean inside = false;
        for (int i = 0; i < ring.size()-1; i++) {
            double[] a = ring.get(i), b = ring.get(i+1);
            if (onSegment(a,b,p)) return allowBoundary;
            if ((a[1] > p[1]) != (b[1] > p[1]) && p[0] < (b[0]-a[0])*(p[1]-a[1])/(b[1]-a[1])+a[0]) inside = !inside;
        } return inside;
    }
    private static boolean ringsIntersect(List<double[]> a, List<double[]> b, boolean properOnly) {
        for (int i = 0; i < a.size()-1; i++) for (int j = 0; j < b.size()-1; j++)
            if (intersects(a.get(i),a.get(i+1),b.get(j),b.get(j+1),properOnly)) return true;
        return false;
    }
    private static double cross(double[] a, double[] b, double[] p) { return (b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0]); }
    private static boolean onSegment(double[] a, double[] b, double[] p) {
        return Math.abs(cross(a,b,p)) <= 1e-10 && p[0] >= Math.min(a[0],b[0])-1e-10 && p[0] <= Math.max(a[0],b[0])+1e-10
                && p[1] >= Math.min(a[1],b[1])-1e-10 && p[1] <= Math.max(a[1],b[1])+1e-10;
    }
    private static boolean intersects(double[] a, double[] b, double[] c, double[] d, boolean properOnly) {
        double ac = cross(a,b,c), ad = cross(a,b,d), ca = cross(c,d,a), cb = cross(c,d,b);
        if (((ac > 1e-10 && ad < -1e-10)||(ac < -1e-10 && ad > 1e-10))
                && ((ca > 1e-10 && cb < -1e-10)||(ca < -1e-10 && cb > 1e-10))) return true;
        return !properOnly && (onSegment(a,b,c)||onSegment(a,b,d)||onSegment(c,d,a)||onSegment(c,d,b));
    }
    private static boolean rectangle(List<double[]> points, double width, double height) {
        if (points.size() != 5) return false;
        Set<String> corners = new HashSet<>();
        for (int i = 0; i < 4; i++) {
            double[] a = points.get(i), b = points.get(i+1);
            if ((a[0] != 0 && a[0] != width) || (a[1] != 0 && a[1] != height)) return false;
            if (a[0] != b[0] && a[1] != b[1]) return false;
            corners.add(a[0] + "/" + a[1]);
        }
        return corners.size() == 4;
    }
}
