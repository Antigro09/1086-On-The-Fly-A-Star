package pathplanning.geometric;

import pathplanning.geometric.Geometry.*;

/** Exact capsule versus axis-aligned rectangle test using a circumscribed robot disk.
 * The disk encloses every robot rotation. Conservative false negatives for narrow passages are intentional. */
public final class SweptCollision {
    private SweptCollision() {}
    public static boolean safe(MapSnapshot map, Pose a, Pose b, double radius, Runnable poll) {
        poll.run();
        if (!inside(map, a, radius) || !inside(map, b, radius)) return false;
        // One-cell closed-boundary padding also covers decimal division rounding at exact tangency.
        int minX = Math.max(0, (int)Math.floor((Math.min(a.x(), b.x()) - radius) / map.cellSize()) - 1);
        int maxX = Math.min(map.columns() - 1, (int)Math.floor((Math.max(a.x(), b.x()) + radius) / map.cellSize()) + 1);
        int minY = Math.max(0, (int)Math.floor((Math.min(a.y(), b.y()) - radius) / map.cellSize()) - 1);
        int maxY = Math.min(map.rows() - 1, (int)Math.floor((Math.max(a.y(), b.y()) + radius) / map.cellSize()) + 1);
        for (int y = minY; y <= maxY; y++) {
            poll.run();
            for (int x = minX; x <= maxX; x++) {
                if ((x & 63) == 0) poll.run();
                if (map.occupied(x, y) && intersects(a, b, new Rectangle(x * map.cellSize(), y * map.cellSize(),
                    Math.min(map.width(), (x + 1) * map.cellSize()), Math.min(map.height(), (y + 1) * map.cellSize())), radius)) return false;
            }
        }
        for (Envelope envelope : map.dynamic()) {
            poll.run();
            if (intersects(a, b, envelope.bounds(), radius)) return false;
        }
        return true;
    }
    private static boolean inside(MapSnapshot map, Pose p, double radius) {
        return p.x() - radius >= 0 && p.y() - radius >= 0 &&
               p.x() + radius <= map.width() && p.y() + radius <= map.height();
    }
    public static boolean intersects(Pose a, Pose b, Rectangle r, double radius) {
        if (pointIn(a, r) || pointIn(b, r)) return true;
        double squared = radius * radius;
        Pose p00 = new Pose(r.minX(), r.minY(), 0), p10 = new Pose(r.maxX(), r.minY(), 0);
        Pose p11 = new Pose(r.maxX(), r.maxY(), 0), p01 = new Pose(r.minX(), r.maxY(), 0);
        return withinOrUncomputable(segmentDistanceSquared(a, b, p00, p10), squared) ||
               withinOrUncomputable(segmentDistanceSquared(a, b, p10, p11), squared) ||
               withinOrUncomputable(segmentDistanceSquared(a, b, p11, p01), squared) ||
               withinOrUncomputable(segmentDistanceSquared(a, b, p01, p00), squared);
    }
    private static boolean withinOrUncomputable(double distanceSquared,double radiusSquared) {
        return !Double.isFinite(distanceSquared) || !Double.isFinite(radiusSquared) || distanceSquared <= radiusSquared;
    }
    private static boolean pointIn(Pose p, Rectangle r) {
        return p.x() >= r.minX() && p.x() <= r.maxX() && p.y() >= r.minY() && p.y() <= r.maxY();
    }
    private static double segmentDistanceSquared(Pose a, Pose b, Pose c, Pose d) {
        double v1 = cross(a, b, c), v2 = cross(a, b, d), v3 = cross(c, d, a), v4 = cross(c, d, b);
        if (((v1 > 0 && v2 < 0) || (v1 < 0 && v2 > 0)) &&
            ((v3 > 0 && v4 < 0) || (v3 < 0 && v4 > 0))) return 0;
        return Math.min(Math.min(pointSegmentSquared(a,c,d), pointSegmentSquared(b,c,d)),
                        Math.min(pointSegmentSquared(c,a,b), pointSegmentSquared(d,a,b)));
    }
    private static double cross(Pose a, Pose b, Pose p) {
        return (b.x()-a.x())*(p.y()-a.y()) - (b.y()-a.y())*(p.x()-a.x());
    }
    private static double pointSegmentSquared(Pose p, Pose a, Pose b) {
        double dx=b.x()-a.x(), dy=b.y()-a.y(), denominator=dx*dx+dy*dy;
        double t=denominator==0 ? 0 : Math.max(0, Math.min(1, ((p.x()-a.x())*dx+(p.y()-a.y())*dy)/denominator));
        double ex=p.x()-a.x()-t*dx, ey=p.y()-a.y()-t*dy;
        return ex*ex+ey*ey;
    }
}
