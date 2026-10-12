package pathplanning.motion;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import pathplanning.geometric.Geometry.Envelope;
import pathplanning.geometric.Geometry.MapSnapshot;
import pathplanning.geometric.Geometry.Pose;
import pathplanning.geometric.Geometry.Rectangle;
import pathplanning.geometric.SweptCollision;
import pathplanning.motion.MotionPath.Bezier;
import pathplanning.motion.MotionPath.Curve;
import pathplanning.motion.MotionPath.Knot;
import pathplanning.motion.MotionPath.Limits;

/** CPU-bounded spline mathematics and collision regressions; no hardware or follower evidence. */
public final class SplineSuite {
    private static final Runnable POLL = () -> {};
    private static final Limits LIMITS = new Limits(3, 2, 2, 3, 4, 0, 0, 0, 0);
    private static int groups, checks;
    private SplineSuite() {}

    public static void main(String[] args) {
        naturalContinuity();
        clampedBoundary();
        nearDuplicateGoal();
        straightRoute();
        broadObstacleBend();
        staticObstacleMap();
        rejectedShortcut();
        cancellationAndStreaming();
        inputIsolationAndBounds();
        System.out.println("SplineSuite: " + groups + " groups, " + checks + " checks passed");
    }

    private static void naturalContinuity() {
        double[][] anchors = {{1, 1}, {2.2, 3.5}, {5, 2.4}, {7.4, 4.2}, {10, 3}};
        List<Bezier> fit = SplineOptimizer.fit(anchors, null);
        require(fit != null && fit.size() == anchors.length - 1, "all natural spline spans exist");
        near(fit.get(0).p0(), p(1, 1), 0, "exact natural start");
        near(last(fit).p3(), p(10, 3), 0, "exact natural end");
        require(norm(fit.get(0).secondDerivative(0)) < 1e-12, "natural start has zero second derivative");
        require(norm(last(fit).secondDerivative(1)) < 1e-12, "natural end has zero second derivative");
        verifyGlobalJoins(fit);
        groups++;
    }

    private static void clampedBoundary() {
        double tangent = Math.PI / 3;
        double[][] anchors = {{1, 1}, {3.6, 2.4}, {6.2, 4}, {9, 3.2}};
        List<Bezier> fit = SplineOptimizer.fit(anchors, tangent);
        require(fit != null, "clamped spline exists");
        double h = distance(fit.get(0).p0(), fit.get(0).p3());
        near(scale(fit.get(0).derivative(0), 1 / h), p(Math.cos(tangent), Math.sin(tangent)), 2e-12,
                "initial travel derivative preserves the supplied tangent in the global parameter");
        near(last(fit).p3(), p(9, 3.2), 0, "clamping does not move the target");
        verifyGlobalJoins(fit);
        List<Curve> accepted = optimize(List.of(p(1, 1), p(3.6, 2.4), p(6.2, 4), p(9, 3.2)),
                map(11, 6, List.of()), .15, tangent);
        require(!accepted.isEmpty(), "clamped candidates can be independently certified");
        for (Curve curve : accepted) {
            Pose d = curve.segments().get(0).derivative(0);
            require(Math.abs(wrap(Math.atan2(d.y(), d.x()) - tangent)) < 2e-12,
                    "optimization never substitutes the chord for measured initial travel direction");
            verifyCertified(curve, map(11, 6, List.of()), .15, p(1, 1), p(9, 3.2));
        }
        groups++;
    }

    private static void straightRoute() {
        MapSnapshot map = map(12, 6, List.of());
        List<Curve> accepted = optimize(List.of(p(1, 2), p(3, 2.4), p(6, 3), p(10, 3.8)), map, .2, null);
        require(!accepted.isEmpty(), "clear straight route yields certified candidates");
        for (Curve curve : accepted) {
            for (Bezier b : curve.segments()) {
                for (Pose control : List.of(b.p0(), b.p1(), b.p2(), b.p3()))
                    require(Math.abs(cross(p(1, 2), p(10, 3.8), control)) < 1e-11,
                            "optimizer does not invent a bend in a clear collinear route");
            }
            require(curve.knots().stream().noneMatch(Knot::stop), "straight spline needs no internal stop");
            verifyCertified(curve, map, .2, p(1, 2), p(10, 3.8));
        }
        groups++;
    }

    private static void nearDuplicateGoal() {
        Pose goal = new Pose(10, 3, -1.2);
        List<Pose> seed = List.of(p(1, 3), p(3, 4.6), p(7, 4.6), p(10, 3 - 5e-9), goal);
        MapSnapshot map = map(12, 6, List.of(obstacle("central-block", 4, 2.5, 6, 3.5)));
        List<Curve> accepted = optimize(seed, map, .25, null);
        require(!accepted.isEmpty(), "near-duplicate last seed retains a feasible detour");
        for (Curve curve : accepted) {
            Pose actual = last(curve.segments()).p3();
            require(actual.x() == goal.x() && actual.y() == goal.y(),
                    "deduplication preserves the exact final goal rather than the nearby preceding pose");
            verifyCertified(curve, map, .25, seed.get(0), goal);
        }
        groups++;
    }

    private static void broadObstacleBend() {
        MapSnapshot map = map(12, 6, List.of(obstacle("central-block", 4, 2.5, 6, 3.5)));
        List<Pose> seed = List.of(p(1, 3), p(3, 4.6), p(7, 4.6), p(10, 3));
        List<Curve> accepted = optimize(seed, map, .25, null);
        require(!accepted.isEmpty(), "detour produces a certified continuous-space spline");
        Curve broad = accepted.stream().filter(c -> c.segments().size() <= 3 &&
                c.segments().stream().allMatch(SplineSuite::broadSpan)).findFirst().orElseThrow(
                        () -> new AssertionError("expected a broad bend over whole spline spans rather than straight gaps and corner fillets"));
        require(broad.segments().stream().allMatch(SplineSuite::broadSpan), "each broad span bends substantially away from its chord");
        require(broad.knots().stream().noneMatch(Knot::stop), "regular broad spline passes joins without stops");
        verifyGlobalJoins(broad.segments());
        for (Curve c : accepted) verifyCertified(c, map, .25, seed.get(0), seed.get(seed.size() - 1));
        groups++;
    }

    private static void staticObstacleMap() {
        boolean[] cells = new boolean[120 * 60];
        for (int y = 25; y < 35; y++) for (int x = 40; x < 60; x++) cells[y * 120 + x] = true;
        MapSnapshot map = new MapSnapshot(12, 6, .1, 120, 60, cells, List.of());
        List<Pose> seed = List.of(p(1, 3), p(3, 4.6), p(7, 4.6), p(10, 3));
        List<Curve> accepted = optimize(seed, map, .2, null);
        require(!accepted.isEmpty(), "static grid obstacles permit a certified broad detour");
        for (Curve c : accepted) verifyCertified(c, map, .2, seed.get(0), seed.get(seed.size() - 1));
        groups++;
    }

    private static void rejectedShortcut() {
        MapSnapshot barrier = map(12, 6, List.of(obstacle("sealed-barrier", 4, 0, 6, 6)));
        require(optimize(List.of(p(1, 3), p(3, 4.6), p(7, 4.6), p(10, 3)), barrier, .2, null).isEmpty(),
                "RDP or proxy optimization cannot publish a shortcut through a sealed obstacle");
        MapSnapshot open = map(12, 6, List.of());
        List<Curve> curve = optimize(List.of(p(1, 3), p(5, 4.6), p(10, 3)), open, .2, null);
        require(!curve.isEmpty(), "independent revalidation fixture exists");
        Bezier segment = curve.get(0).segments().get(0);
        Pose inside = segment.at(.5);
        MapSnapshot changed = map(12, 6, List.of(obstacle("changed-interior", inside.x() - .03, inside.y() - .03,
                inside.x() + .03, inside.y() + .03)));
        require(!BezierSmoother.revalidate(curve.get(0), changed, .2, POLL),
                "new obstacle inside an earlier broad span invalidates exact controls");
        groups++;
    }

    private static void cancellationAndStreaming() {
        List<Pose> seed = List.of(p(1, 3), p(3, 4.6), p(7, 4.6), p(10, 3));
        MapSnapshot map = map(12, 6, List.of(obstacle("central-block", 4, 2.5, 6, 3.5)));
        AtomicInteger polls = new AtomicInteger();
        RuntimeException marker = new RuntimeException("bounded-cancellation");
        List<Curve> accepted = new ArrayList<>();
        boolean same = false;
        try {
            SplineOptimizer.visitCandidates(seed, map, .25, null, LIMITS,
                    () -> { if (polls.incrementAndGet() == 128) throw marker; }, accepted::add);
        } catch (RuntimeException stopped) { same = stopped == marker; }
        require(same && polls.get() == 128, "mid-optimization poll throws the exact cancellation without further work");
        require(accepted.isEmpty(), "partial proxy/control work has no certified publication authority");
        List<Curve> incumbent = new ArrayList<>();
        RuntimeException budget = new RuntimeException("refinement-budget");
        same = false;
        try {
            SplineOptimizer.visitCandidates(seed, map, .25, null, LIMITS,
                    () -> { if (!incumbent.isEmpty()) throw budget; }, incumbent::add);
        } catch (RuntimeException stopped) { same = stopped == budget; }
        require(same && incumbent.size() == 1, "first fully certified candidate is delivered before refinement runs out of budget");
        verifyCertified(incumbent.get(0), map, .25, seed.get(0), seed.get(seed.size() - 1));
        groups++;
    }

    private static void inputIsolationAndBounds() {
        List<Pose> seed = new ArrayList<>(List.of(new Pose(1, 3, .7), p(3, 4.6), p(7, 4.6), new Pose(10, 3, -1.2)));
        List<Pose> before = List.copyOf(seed);
        List<Envelope> obstacles = new ArrayList<>(List.of(obstacle("central-block", 4, 2.5, 6, 3.5)));
        boolean[] cells = new boolean[120 * 60]; cells[55 * 120 + 110] = true;
        boolean[] original = cells.clone();
        MapSnapshot map = new MapSnapshot(12, 6, .1, 120, 60, cells, obstacles);
        List<Envelope> snapshotObstacles = map.dynamic();
        List<Curve> accepted = optimize(seed, map, .25, null);
        require(!accepted.isEmpty(), "immutability fixture is optimized");
        require(seed.equals(before), "optimizer does not move or rewrite caller seed poses/headings");
        require(obstacles.equals(snapshotObstacles) && map.dynamic().equals(snapshotObstacles), "optimizer does not replace raw obstacle identity/dimensions");
        boolean unchanged = true;
        for (int y = 0; y < 60; y++) for (int x = 0; x < 120; x++)
            unchanged &= map.occupied(x, y) == original[y * 120 + x] && cells[y * 120 + x] == original[y * 120 + x];
        require(unchanged, "static snapshot and its source grid remain unchanged");
        double[][] coordinates = {{1, 1}, {3, 4}, {6, 2}};
        double[][] held = {{1, 1}, {3, 4}, {6, 2}};
        require(SplineOptimizer.fit(coordinates, null) != null, "fit immutability fixture exists");
        for (int i = 0; i < coordinates.length; i++)
            require(coordinates[i][0] == held[i][0] && coordinates[i][1] == held[i][1], "fit does not mutate its anchor array");
        boolean immutable = false;
        try { accepted.get(0).segments().clear(); } catch (UnsupportedOperationException expected) { immutable = true; }
        require(immutable, "published spline controls are immutable");
        require(SplineOptimizer.fit(new double[][] {{1, 1}, {1.01, 1}}, null) == null,
                "near-coincident anchors are rejected rather than generating unstable derivative scales");
        require(SplineOptimizer.fit(new double[49][2], null) == null, "fit anchor capacity is bounded");
        groups++;
    }

    private static List<Curve> optimize(List<Pose> seed, MapSnapshot map, double radius, Double tangent) {
        List<Curve> output = new ArrayList<>();
        SplineOptimizer.visitCandidates(seed, map, radius, tangent, LIMITS, POLL, output::add);
        return output;
    }

    private static void verifyGlobalJoins(List<Bezier> curves) {
        for (int i = 1; i < curves.size(); i++) {
            Bezier left = curves.get(i - 1), right = curves.get(i);
            double hLeft = distance(left.p0(), left.p3()), hRight = distance(right.p0(), right.p3());
            near(left.p3(), right.p0(), 0, "spline spans have no gap");
            near(scale(left.derivative(1), 1 / hLeft), scale(right.derivative(0), 1 / hRight), 2e-11,
                    "first derivatives agree in the common chord-length parameter");
            near(scale(left.secondDerivative(1), 1 / (hLeft * hLeft)), scale(right.secondDerivative(0), 1 / (hRight * hRight)), 2e-11,
                    "second derivatives agree in the common chord-length parameter");
            require(Math.abs(curvature(left, 1) - curvature(right, 0)) < 2e-11,
                    "signed geometric curvature is continuous at a regular join");
        }
    }

    private static void verifyCertified(Curve curve, MapSnapshot map, double radius, Pose start, Pose end) {
        require(curve.segments().size() <= 128 && curve.knots().size() <= 1024, "certificate storage is bounded");
        near(curve.segments().get(0).p0(), start, 0, "accepted spline retains exact start position");
        near(last(curve.segments()).p3(), end, 0, "accepted spline retains exact end position");
        require(BezierSmoother.revalidate(curve, map, radius, POLL), "independent exact-control revalidation succeeds");
        double previous = -1;
        for (Knot knot : curve.knots()) {
            require(Double.isFinite(knot.distanceM()) && knot.distanceM() > previous &&
                    Double.isFinite(knot.curvatureBound()) && knot.curvatureBound() >= 0,
                    "certified arc coordinates increase and whole-interval bounds are finite");
            previous = knot.distanceM();
        }
        boolean safe = true, regular = true;
        for (Bezier segment : curve.segments()) for (int i = 0; i <= 64; i++) {
            double u = (double) i / 64;
            Pose p = segment.at(u), d = segment.derivative(u);
            regular &= Double.isFinite(norm(d)) && norm(d) > 1e-7;
            safe &= SweptCollision.safe(map, p, p, radius, POLL);
        }
        require(regular, "dense independent derivative checks find no cusp or stationary interior");
        require(safe, "dense independent footprint occupancy checks agree with the full certificate");
    }

    private static boolean broadSpan(Bezier b) {
        double chord = distance(b.p0(), b.p3());
        if (chord < 1) return false;
        double maxDeviation = 0;
        for (int i = 1; i < 8; i++) maxDeviation = Math.max(maxDeviation, Math.abs(cross(b.p0(), b.p3(), b.at((double) i / 8))) / chord);
        return maxDeviation > .08;
    }
    private static double curvature(Bezier b, double u) {
        Pose d = b.derivative(u), a = b.secondDerivative(u); double speed = norm(d);
        require(speed > 1e-8, "join is regular before comparing signed curvature");
        return (d.x() * a.y() - d.y() * a.x()) / (speed * speed * speed);
    }
    private static MapSnapshot map(double width, double height, List<Envelope> obstacles) {
        double cell = .1; int columns = (int) Math.ceil(width / cell), rows = (int) Math.ceil(height / cell);
        return new MapSnapshot(width, height, cell, columns, rows, new boolean[columns * rows], obstacles);
    }
    private static Envelope obstacle(String id, double x0, double y0, double x1, double y1) {
        return new Envelope(id, new Rectangle(x0, y0, x1, y1), 0);
    }
    private static Pose p(double x, double y) { return new Pose(x, y, 0); }
    private static Pose scale(Pose p, double factor) { return p(p.x() * factor, p.y() * factor); }
    private static Bezier last(List<Bezier> curves) { return curves.get(curves.size() - 1); }
    private static double norm(Pose p) { return Math.hypot(p.x(), p.y()); }
    private static double distance(Pose a, Pose b) { return Math.hypot(a.x() - b.x(), a.y() - b.y()); }
    private static double cross(Pose a, Pose b, Pose c) { return (b.x() - a.x()) * (c.y() - a.y()) - (b.y() - a.y()) * (c.x() - a.x()); }
    private static double wrap(double x) { return Math.atan2(Math.sin(x), Math.cos(x)); }
    private static void near(Pose actual, Pose expected, double tolerance, String message) { require(distance(actual, expected) <= tolerance, message); }
    private static void require(boolean value, String message) { if (!value) throw new AssertionError(message); checks++; }
}
