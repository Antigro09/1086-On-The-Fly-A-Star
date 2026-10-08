package pathplanning.geometric;

import java.util.List;

/** Controller-independent implementation values. World-State owns the public request contract. */
public final class Geometry {
    private Geometry() {}

    public record Pose(double x, double y, double headingRadians) {}
    public record Rectangle(double minX, double minY, double maxX, double maxY) {}
    /** Raw robot dimensions; this solver is the only footprint inflation owner. */
    public record Footprint(double lengthMeters, double widthMeters, double clearanceMeters) {
        public double enclosingRadius() { return Math.hypot(lengthMeters, widthMeters) / 2 + clearanceMeters; }
    }
    /** Union envelope already bounds an object's dimensions, forecast, uncertainty and age.
     * It MUST NOT contain robot footprint inflation. This solver adds that exactly once. */
    // -1 explicitly means the upstream API omitted age; an enclosing validity envelope is required there.
    public record Envelope(String objectId, Rectangle bounds, long observationAgeMicros) {}
    public record Constraints(double maxSpeed, double maxAcceleration, double maxAngularSpeed,
                              double startSpeed, double endSpeed, double positionTolerance,
                              double headingToleranceRadians) {}
    public record Budget(long timeoutNanos, int maxExpanded, int maxQueueEntries, int maxCells) {}

    /** Immutable per-request raw occupancy snapshot. No detection-frame update method exists. */
    public static final class MapSnapshot {
        private final double width, height, cellSize;
        private final int columns, rows;
        private final boolean[] occupied;
        private final boolean hasStaticOccupancy;
        private final List<Envelope> dynamic;
        public MapSnapshot(double width, double height, double cellSize, int columns, int rows,
                           boolean[] occupied, List<Envelope> dynamic) {
            this.width = width; this.height = height; this.cellSize = cellSize;
            this.columns = columns; this.rows = rows;
            this.occupied = occupied == null ? null : occupied.clone();
            boolean any = false;
            if (this.occupied != null) for (boolean cell : this.occupied) if (cell) { any = true; break; }
            this.hasStaticOccupancy = any;
            this.dynamic = dynamic == null ? null : List.copyOf(dynamic);
        }
        public double width() { return width; }
        public double height() { return height; }
        public double cellSize() { return cellSize; }
        public int columns() { return columns; }
        public int rows() { return rows; }
        public List<Envelope> dynamic() { return dynamic; }
        public int occupancySize() { return occupied == null ? -1 : occupied.length; }
        /** Cached from the owned immutable copy; dynamic envelopes remain independent. */
        public boolean hasStaticOccupancy() { return hasStaticOccupancy; }
        public boolean occupied(int x, int y) { return occupied[y * columns + x]; }
    }
    public record Input(Pose start, Pose goal, Footprint footprint, Constraints constraints,
                        MapSnapshot map, Budget budget, boolean simplify) {}
    public enum Status { SUCCESS, INVALID_INPUT, UNSAFE_START, UNSAFE_GOAL, NO_PATH,
        TIMEOUT, RESOURCE_LIMIT, CANCELLED, COLLISION_VALIDATION_FAILED }
    public record Output(Status status, List<Pose> poses, long planningDurationNanos,
                         int expandedNodes, String detail) {
        public Output { poses = List.copyOf(poses); }
    }
    @FunctionalInterface public interface Cancellation { boolean cancelled(); }
}
