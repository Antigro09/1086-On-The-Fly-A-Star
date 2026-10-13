package pathplanning.motion;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CancellationException;
import java.util.concurrent.atomic.AtomicInteger;
import pathplanning.geometric.Geometry.MapSnapshot;
import pathplanning.geometric.Geometry.Pose;
import pathplanning.motion.MotionPath.*;

/** Bounded CPU tests of motion mathematics; no controller, simulator or hardware qualification. */
public final class TimingSuite {
    private static int passed;
    private TimingSuite() {}
    public static void main(String[] args) {
        run("analytic straight triangular duration", TimingSuite::triangular);
        run("analytic straight trapezoid and anticipatory braking", TimingSuite::trapezoid);
        run("nonzero boundary speeds preserved or rejected", TimingSuite::boundaries);
        run("curvature tightening and total acceleration norm", TimingSuite::curvature);
        run("C1 joins flow without stops; sharp joins stop", TimingSuite::joins);
        run("holonomic heading independent of strafe/reverse tangent", TimingSuite::holonomic);
        run("independent angular bounds and rest-to-rest synchronization", TimingSuite::heading);
        run("moving start preserves fastest translation then holds goal", TimingSuite::goalHold);
        run("stopped start delays departure and preserves moving endpoint", TimingSuite::departureDelay);
        run("nonzero initial spin preserved with final heading hold", TimingSuite::initialSpin);
        run("opposite spin and angular overshoot return", TimingSuite::spinReversal);
        run("angular endpoint limits and bounded winding", TimingSuite::angularBoundaries);
        run("stationary rotation and zero-duration stationary path", TimingSuite::stationary);
        run("integrated inverse arc length, finite and monotonic samples", TimingSuite::inverse);
        run("canonical arc coordinates match physical velocity", TimingSuite::arcCoordinates);
        run("invalid finite bounds, malformed curve and cancellation", TimingSuite::invalid);
        System.out.println("PASS: " + passed + " motion timing groups; bounded CPU mathematics only; no hardware execution");
    }
    private static Limits limits(double speed, double acceleration) {
        return new Limits(speed, acceleration, acceleration, 3, 4, 0, 0, 0, 0);
    }
    private static Profile profile(Curve curve, Limits limits) { return TimeParameterizer.profile(curve, limits, () -> {}); }
    private static Pose p(double x, double y) { return new Pose(x, y, 0); }
    private static Bezier line(Pose a, Pose b) {
        return new Bezier(a, p(a.x() + (b.x() - a.x()) / 3, a.y() + (b.y() - a.y()) / 3),
                p(a.x() + 2 * (b.x() - a.x()) / 3, a.y() + 2 * (b.y() - a.y()) / 3), b);
    }
    private static Curve straight(double distance) { return fromSegments(List.of(line(p(1, 1), p(1 + distance, 1))), List.of(false), false); }
    private static Curve fromSegments(List<Bezier> segments, List<Boolean> stops, boolean dense) {
        List<Knot> knots = new ArrayList<>(); double distance = 0;
        Pose first = segments.get(0).p0(), d = segments.get(0).derivative(0);
        knots.add(new Knot(0, 0, 0, first.x(), first.y(), Math.atan2(d.y(), d.x()), 0, false));
        for (int s = 0; s < segments.size(); s++) {
            Bezier b = segments.get(s); int count = dense ? 64 : 1;
            for (int i = 1; i <= count; i++) {
                double from = (double)(i - 1) / count, to = (double)i / count;
                distance += integrate(b, from, to); Pose point = b.at(to), v = b.derivative(to);
                double max = 0;
                for (int j = 0; j <= 20; j++) max = Math.max(max, curvature(b, from + (to - from) * j / 20));
                // Test-only upper bound is intentionally loose; production smoothing uses certificates.
                double bound = max == 0 ? 0 : max * 1.25 + .01;
                knots.add(new Knot(distance, s, to, point.x(), point.y(), Math.atan2(v.y(), v.x()), bound, i == count && stops.get(s)));
            }
        }
        return new Curve(segments, knots, distance, "Synthetic timing mathematics fixture");
    }
    private static double integrate(Bezier b, double from, double to) {
        int count = 200; double sum = 0, h = (to - from) / count;
        for (int i = 0; i <= count; i++) { Pose v = b.derivative(from + i * h); sum += (i == 0 || i == count ? 1 : i % 2 == 0 ? 2 : 4) * Math.hypot(v.x(), v.y()); }
        return sum * h / 3;
    }
    private static double curvature(Bezier b, double u) {
        Pose v = b.derivative(u), a = b.secondDerivative(u); double norm = Math.hypot(v.x(), v.y());
        return Math.abs(v.x() * a.y() - v.y() * a.x()) / (norm * norm * norm);
    }
    private static void triangular() {
        Curve c = straight(1); Limits l = limits(3, 2); Profile p = profile(c, l);
        close(p.durationSec(), Math.sqrt(2), 1e-10, "Triangular duration");
        close(p.peakSpeedMps(), Math.sqrt(2), 1e-10, "Triangular peak");
        Sample middle = TimeParameterizer.atTime(c, p, p.durationSec() / 2);
        close(middle.distanceM(), .5, 1e-10, "Triangle midpoint distance");
        checkDense(c, p, l, 300);
    }
    private static void trapezoid() {
        Curve c = straight(12); Limits l = limits(2, 1); Profile p = profile(c, l);
        close(p.durationSec(), 8, 1e-10, "Trapezoidal duration");
        close(TimeParameterizer.atTime(c, p, 1).speedMps(), 1, 1e-10, "Acceleration ramp");
        close(TimeParameterizer.atTime(c, p, 4).speedMps(), 2, 1e-10, "Cruise plateau");
        close(TimeParameterizer.atTime(c, p, 7).speedMps(), 1, 1e-10, "Anticipated braking");
        close(TimeParameterizer.atTime(c, p, 7).distanceM(), 11.5, 1e-10, "Braking distance");
        checkDense(c, p, l, 300);
    }
    private static void boundaries() {
        Curve c = straight(4); Limits l = new Limits(3, 2, 2, 3, 4, 1, .5, 0, 0); Profile p = profile(c, l);
        close(p.samples().get(0).speedMps(), 1, 0, "Start speed altered");
        close(p.samples().get(p.samples().size() - 1).speedMps(), .5, 0, "End speed altered");
        close(p.durationSec(), (2 * Math.sqrt(8.625) - 1.5) / 2, 1e-10, "Moving endpoint duration");
        checkDense(c, p, l, 250);
        rejects(() -> profile(straight(.5), new Limits(3, 1, 1, 3, 4, 3, 0, 0, 0)), "Infeasible braking silently zeroed start");
        rejects(() -> profile(straight(.5), new Limits(3, 1, 1, 3, 4, 0, 3, 0, 0)), "Infeasible acceleration silently zeroed end");
        rejects(() -> profile(straight(1), new Limits(3, 2, 2, .2, .2, 1, 1, 0, Math.PI)), "Unsupported moving-boundary angular synchronization was hidden");
    }
    private static Curve bend(double scale) {
        return fromSegments(List.of(new Bezier(p(1, 1), p(1 + scale, 1), p(1 + scale, 1 + scale), p(1, 1 + scale))), List.of(false), true);
    }
    private static void curvature() {
        Curve c = bend(1); Limits l = new Limits(4, 2, 1, 3, 4, 0, 0, 0, 0); Profile p = profile(c, l);
        require(p.peakSpeedMps() < 1.2, "Tight curve ignored centripetal bound"); checkDense(c, p, l, 500);
        Curve broader = bend(3); Profile broad = profile(broader, l);
        require(broad.peakSpeedMps() > p.peakSpeedMps(), "Tighter curvature did not reduce speed");
        Limits combined = new Limits(4, 1, 10, 3, 4, 0, 0, 0, 0);
        checkDense(c, profile(c, combined), combined, 500);
    }
    private static MapSnapshot map() { return new MapSnapshot(10, 10, .1, 100, 100, new boolean[10000], List.of()); }
    private static void joins() {
        List<Curve> choices = BezierSmoother.candidates(List.of(p(1, 1), p(5, 1), p(5, 5)), map(), .2, null, () -> {});
        require(!choices.isEmpty(), "Smoother supplied no candidates");
        Curve rounded = choices.get(0); Profile roundedProfile = profile(rounded, limits(3, 2));
        require(rounded.knots().stream().noneMatch(Knot::stop), "C1 rounded join forced a stop");
        for (Knot k : rounded.knots()) if (k.u() == 1 && k.segment() < rounded.segments().size() - 1)
            require(speedAtDistance(roundedProfile, k.distanceM()) > .1, "Continuous join lost positive speed");
        checkDense(rounded, roundedProfile, limits(3, 2), 400);
        Curve sharp = choices.get(choices.size() - 1); Profile sharpProfile = profile(sharp, limits(3, 2));
        require(sharp.knots().stream().anyMatch(Knot::stop), "Sharp fallback lacked required stop");
        for (Knot k : sharp.knots()) if (k.stop()) close(speedAtDistance(sharpProfile, k.distanceM()), 0, 1e-10, "Sharp join did not stop");
    }
    private static double speedAtDistance(Profile profile, double distance) {
        for (Sample s : profile.samples()) if (Math.abs(s.distanceM() - distance) < 1e-8) return s.speedMps();
        throw new AssertionError("Knot timing boundary absent");
    }
    private static void holonomic() {
        Curve strafe = fromSegments(List.of(line(p(1, 1), p(1, 4))), List.of(false), false); Profile p = profile(strafe, limits(2, 2));
        Sample s = TimeParameterizer.atTime(strafe, p, p.durationSec() / 2);
        close(s.headingRadians(), 0, 1e-12, "Strafe heading became tangent"); require(s.vyMps() > 0 && Math.abs(s.vxMps()) < 1e-9, "Strafe velocity direction");
        Curve reverse = fromSegments(List.of(line(p(4, 1), p(1, 1))), List.of(false), false); Profile r = profile(reverse, limits(2, 2));
        Sample back = TimeParameterizer.atTime(reverse, r, r.durationSec() / 2);
        close(back.headingRadians(), 0, 1e-12, "Reverse heading changed"); require(back.vxMps() < 0, "Reverse field velocity");
    }
    private static void heading() {
        Curve c = straight(.2); Limits l = new Limits(2, 2, 2, 1, 2, 0, 0, 0, Math.PI); Profile p = profile(c, l);
        close(p.durationSec(), Math.PI + .5, 1e-10, "Angular trapezoid controls synchronized duration");
        checkDense(c, p, l, 500);
        Sample last = p.samples().get(p.samples().size() - 1);
        close(Math.atan2(Math.sin(last.headingRadians() - Math.PI), Math.cos(last.headingRadians() - Math.PI)), 0, 1e-10, "Final holonomic heading");
        Curve longLine = straight(10); Limits shortest = new Limits(2, 2, 2, 1, 2, 0, 0, 3, -3);
        Profile wrapped = profile(longLine, shortest); checkDense(longLine, wrapped, shortest, 300);
        require(wrapped.samples().stream().mapToDouble(s -> Math.abs(s.omegaRadPerSec())).max().orElseThrow() < .2, "Heading took a full-turn shortcut");
    }
    private static void goalHold() {
        Curve c = straight(1); Limits translating = new Limits(2, 2, 2, 1, 2, 1, 0, 0, 0);
        Profile fastest = profile(c, translating);
        Limits synchronizedLimits = new Limits(2, 2, 2, 1, 2, 1, 0, 0, Math.PI);
        Profile p = profile(c, synchronizedLimits);
        close(p.durationSec(), Math.PI + .5, 1e-10, "Rotation sets combined duration");
        close(p.samples().get(0).speedMps(), 1, 0, "Moving initial velocity changed");
        Sample half = TimeParameterizer.atTime(c, p, fastest.durationSec() / 2);
        close(half.distanceM(), TimeParameterizer.atTime(c, fastest, fastest.durationSec() / 2).distanceM(), 1e-10, "Translation was slowed instead of preserved");
        Sample holding = TimeParameterizer.atTime(c, p, (fastest.durationSec() + p.durationSec()) / 2);
        close(holding.distanceM(), c.lengthM(), 1e-12, "Goal hold distance");
        close(holding.speedMps(), 0, 0, "Goal hold kept linear velocity");
        require(Math.abs(holding.omegaRadPerSec()) > 0, "Independent rotation did not continue at the goal");
        checkDense(c, p, synchronizedLimits, 500);
    }
    private static void departureDelay() {
        Curve c = straight(1); Limits translating = new Limits(2, 2, 2, 1, 2, 0, 1, 0, 0);
        Profile fastest = profile(c, translating);
        Limits synchronizedLimits = new Limits(2, 2, 2, 1, 2, 0, 1, 0, Math.PI);
        Profile p = profile(c, synchronizedLimits); double delay = p.durationSec() - fastest.durationSec();
        Sample waiting = TimeParameterizer.atTime(c, p, delay / 2);
        close(waiting.distanceM(), 0, 0, "Delayed departure moved early");
        close(waiting.speedMps(), 0, 0, "Delayed departure did not wait at rest");
        require(Math.abs(waiting.omegaRadPerSec()) > 0, "Heading did not rotate during departure delay");
        Sample middle = TimeParameterizer.atTime(c, p, delay + fastest.durationSec() / 2);
        close(middle.distanceM(), TimeParameterizer.atTime(c, fastest, fastest.durationSec() / 2).distanceM(), 1e-10, "Delayed translation was altered");
        Sample last = p.samples().get(p.samples().size() - 1);
        close(last.speedMps(), 1, 0, "Moving final velocity changed"); close(last.distanceM(), c.lengthM(), 1e-12, "Moving endpoint distance");
        checkDense(c, p, synchronizedLimits, 500);
    }
    private static void initialSpin() {
        Curve c = straight(8); Limits l = new Limits(2, 2, 2, 2, 3, 0, 0, 0, Math.PI / 2, .8, 0); Profile p = profile(c, l);
        close(p.samples().get(0).omegaRadPerSec(), .8, 0, "Initial spin was zeroed or stretched");
        close(p.samples().get(p.samples().size() - 1).omegaRadPerSec(), 0, 0, "Final angular velocity");
        checkDense(c, p, l, 400);
        close(TimeParameterizer.atTime(c, p, p.durationSec() * .9).headingRadians(), Math.PI / 2, 1e-8, "Final heading not held while translation continues");
    }
    private static void spinReversal() {
        Curve c = still(); Limits opposite = new Limits(2, 2, 2, 2, 3, 0, 0, 0, Math.PI / 2, -.8, 0);
        Profile o = profile(c, opposite); close(o.samples().get(0).omegaRadPerSec(), -.8, 0, "Opposite initial spin changed");
        require(TimeParameterizer.atTime(c, o, .05).headingRadians() < 0, "Opposite spin did not initially move in its existing direction");
        checkDense(c, o, opposite, 500);
        Limits overshoot = new Limits(2, 2, 2, 2, 2, 0, 0, 0, .1, 1.2, 0); Profile p = profile(c, overshoot);
        require(TimeParameterizer.atTime(c, p, .6).headingRadians() > .35, "Required angular overshoot was hidden");
        require(p.samples().stream().anyMatch(s -> s.omegaRadPerSec() < 0), "Overshoot did not reverse");
        close(p.samples().get(p.samples().size() - 1).headingRadians(), .1, 1e-10, "Overshoot return heading");
        checkDense(c, p, overshoot, 500);
    }
    private static void angularBoundaries() {
        Curve c = still(); Limits loop = new Limits(2, 2, 2, 2, 1, 0, 0, 0, 0, 2, 0); Profile p = profile(c, loop);
        close(p.samples().get(0).omegaRadPerSec(), 2, 0, "Periodic candidate initial speed");
        require(p.durationSec() < 4.3, "Adjacent full-turn candidate was not considered");
        checkDense(c, p, loop, 600);
        Curve line = straight(8); Limits finishMoving = new Limits(2, 2, 2, 2, 3, 0, 0, 0, 1, 0, .5); Profile moving = profile(line, finishMoving);
        close(moving.samples().get(moving.samples().size() - 1).omegaRadPerSec(), .5, 0, "Nonzero terminal spin changed");
        checkDense(line, moving, finishMoving, 400);
        rejects(() -> profile(c, new Limits(2, 2, 2, 1, 2, 0, 0, 0, 1, 1.1, 0)), "Excess initial omega accepted");
        rejects(() -> profile(c, new Limits(2, 2, 2, 1, 2, 0, 0, 0, 1, Double.NaN, 0)), "Nonfinite initial omega accepted");
        rejects(() -> profile(c, new Limits(2, 2, 2, 1e-9, 1e6, 0, 0, 0, .001)), "Unrepresentable tiny angular end phase accepted");
    }
    private static Curve still() { return new Curve(List.of(), List.of(new Knot(0, 0, 0, 2, 3, 0, 0, false)), 0, "Stationary fixture"); }
    private static void stationary() {
        Curve c = still(); Limits rotation = new Limits(2, 2, 2, 1, 2, 0, 0, 0, Math.PI);
        Profile p = profile(c, rotation); close(p.durationSec(), Math.PI + .5, 1e-10, "In-place rotation duration");
        checkDense(c, p, rotation, 300);
        for (Sample s : p.samples()) require(s.speedMps() == 0 && s.x() == 2 && s.y() == 3, "Rotation translated robot");
        Profile noMotion = profile(c, limits(2, 2)); require(noMotion.durationSec() == 0 && noMotion.samples().size() == 1, "Zero-length stationary timing");
        rejects(() -> profile(c, new Limits(2, 2, 2, 1, 2, .1, 0, 0, 0)), "Stationary start velocity was zeroed");
    }
    private static void inverse() {
        Curve c = bend(1); Profile p = profile(c, limits(2, 2));
        for (int i = 1; i < 100; i++) {
            Sample s = TimeParameterizer.atTime(c, p, p.durationSec() * i / 100);
            Bezier b = c.segments().get(s.segment()); Pose evaluated = b.at(s.u());
            close(s.x(), evaluated.x(), 1e-12, "Position was a chord approximation"); close(s.y(), evaluated.y(), 1e-12, "Position was a chord approximation");
            close(integrate(b, 0, s.u()), s.distanceM(), 2e-7, "Distance-to-parameter inversion");
        }
        require(TimeParameterizer.atTime(c, p, -1).timeSec() == 0, "Before-start sample not clamped");
        require(TimeParameterizer.atTime(c, p, p.durationSec() + 1).timeSec() == p.durationSec(), "After-end sample not clamped");
        rejects(() -> TimeParameterizer.atTime(c, p, Double.NaN), "NaN time accepted");
    }
    private static void arcCoordinates() {
        Curve c = straight(1); List<Knot> distorted = new ArrayList<>();
        for (Knot k : c.knots()) distorted.add(new Knot(k.distanceM() * .999995, k.segment(), k.u(), k.x(), k.y(), k.tangentRadians(), k.curvatureBound(), k.stop()));
        Curve estimated = new Curve(c.segments(), distorted, .999995, "Biased source quadrature");
        rejects(() -> profile(estimated, limits(.2, 1)), "Biased metadata silently changed physical speed");
        Curve integrated = TimeParameterizer.integratedCurve(estimated, () -> {}); close(integrated.lengthM(), 1, 1e-12, "Canonical physical arc length");
        Profile p = profile(integrated, limits(.2, 1)); double t = p.durationSec() / 2, h = 1e-5;
        Sample a = TimeParameterizer.atTime(integrated, p, t - h), b = TimeParameterizer.atTime(integrated, p, t + h);
        close((b.x() - a.x()) / (2 * h), .2, 2e-7, "Physical cruise speed differs from reported speed");
        double accelerationTime = .1, step = .01;
        double lowX = TimeParameterizer.atTime(integrated, p, accelerationTime - step).x();
        double middleX = TimeParameterizer.atTime(integrated, p, accelerationTime).x();
        double highX = TimeParameterizer.atTime(integrated, p, accelerationTime + step).x();
        close((highX - 2 * middleX + lowX) / (step * step), 1, 2e-6, "Physical acceleration differs from its limit");
        Curve turning = TimeParameterizer.integratedCurve(bend(1), () -> {}); Profile curved = profile(turning, limits(2, 2));
        for (int i = 1; i < 20; i++) {
            double time = curved.durationSec() * i / 20; Sample low = TimeParameterizer.atTime(turning, curved, time - h), high = TimeParameterizer.atTime(turning, curved, time + h), middle = TimeParameterizer.atTime(turning, curved, time);
            close((high.x() - low.x()) / (2 * h), middle.vxMps(), 3e-6, "Physical x velocity mismatch");
            close((high.y() - low.y()) / (2 * h), middle.vyMps(), 3e-6, "Physical y velocity mismatch");
        }
    }
    private static void invalid() {
        Curve c = straight(1);
        rejects(() -> profile(c, new Limits(Double.NaN, 2, 2, 3, 4, 0, 0, 0, 0)), "NaN limit");
        rejects(() -> profile(c, new Limits(2, Double.POSITIVE_INFINITY, 2, 3, 4, 0, 0, 0, 0)), "Infinite limit");
        rejects(() -> profile(c, new Limits(2, 0, 2, 3, 4, 0, 0, 0, 0)), "Zero acceleration");
        List<Knot> bad = new ArrayList<>(c.knots()); Knot k = bad.get(1); bad.set(1, new Knot(.1, k.segment(), k.u(), k.x(), k.y(), k.tangentRadians(), 0, false));
        rejects(() -> profile(new Curve(c.segments(), bad, .1, "Incorrect arc metadata"), limits(2, 2)), "False distance metadata");
        Curve turning = bend(1); List<Knot> underestimated = new ArrayList<>();
        for (Knot node : turning.knots()) underestimated.add(new Knot(node.distanceM(), node.segment(), node.u(), node.x(), node.y(), node.tangentRadians(), 0, node.stop()));
        rejects(() -> profile(new Curve(turning.segments(), underestimated, turning.lengthM(), "False curvature"), limits(2, 2)), "Understated curvature accepted");
        Curve kink = fromSegments(List.of(line(p(1, 1), p(2, 1)), line(p(2, 1), p(2, 2))), List.of(false, false), false);
        rejects(() -> profile(kink, limits(2, 2)), "Sharp join without stop accepted");
        Curve gap = fromSegments(List.of(line(p(1, 1), p(2, 1)), line(p(3, 1), p(4, 1))), List.of(false, false), false);
        rejects(() -> profile(gap, limits(2, 2)), "Disconnected Bézier geometry accepted");
        AtomicInteger calls = new AtomicInteger();
        try { TimeParameterizer.profile(bend(1), limits(2, 2), () -> { if (calls.incrementAndGet() >= 10) throw new CancellationException("Synthetic poll cancellation"); }); throw new AssertionError("Cancellation ignored"); }
        catch (CancellationException expected) { require(calls.get() == 10, "Cancellation was swallowed"); }
        try { TimeParameterizer.integratedCurve(still(), () -> { throw new CancellationException("Pre-cancelled normalization"); }); throw new AssertionError("Stationary normalization ignored cancellation"); }
        catch (CancellationException expected) { /* immediate poll even with no geometry work */ }
    }
    private static void checkDense(Curve c, Profile p, Limits l, int count) {
        double previousTime = -1;
        for (Sample s : p.samples()) { require(s.timeSec() > previousTime, "Timing samples not strictly increasing"); previousTime = s.timeSec(); }
        Sample previous = null; double dt = p.durationSec() / count;
        for (int i = 0; i <= count; i++) {
            Sample s = TimeParameterizer.atTime(c, p, dt * i);
            for (double v : new double[]{s.timeSec(), s.distanceM(), s.u(), s.x(), s.y(), s.headingRadians(), s.tangentRadians(), s.speedMps(), s.accelerationMps2(), s.vxMps(), s.vyMps(), s.omegaRadPerSec()}) require(Double.isFinite(v), "Nonfinite timed sample");
            require(s.speedMps() >= 0 && s.speedMps() <= l.maxSpeed() + 1e-8, "Linear speed bound");
            double k = c.segments().isEmpty() ? 0 : curvature(c.segments().get(s.segment()), s.u());
            require(k * s.speedMps() * s.speedMps() <= l.maxCentripetalAcceleration() + 1e-6, "Centripetal bound");
            require(Math.hypot(s.accelerationMps2(), k * s.speedMps() * s.speedMps()) <= l.maxAcceleration() + 1e-6, "TOTAL linear acceleration bound");
            require(Math.abs(s.omegaRadPerSec()) <= l.maxAngularSpeed() + 1e-8, "Angular speed bound");
            close(Math.hypot(s.vxMps(), s.vyMps()), s.speedMps(), 1e-9, "Field velocity norm");
            if (previous != null) {
                require(s.distanceM() + 1e-10 >= previous.distanceM(), "Arc distance went backwards");
                require(Math.abs(s.omegaRadPerSec() - previous.omegaRadPerSec()) / dt <= l.maxAngularAcceleration() + 1e-5, "Angular acceleration bound");
            }
            previous = s;
        }
        for (int i = 1; i < p.samples().size(); i++) {
            Sample a = p.samples().get(i - 1), b = p.samples().get(i); double span = b.timeSec() - a.timeSec();
            close(b.distanceM() - a.distanceM(), (a.speedMps() + b.speedMps()) * span / 2, 1e-8, "Integrated timing interval distance");
            double turn = Math.atan2(Math.sin(b.headingRadians() - a.headingRadians()), Math.cos(b.headingRadians() - a.headingRadians()));
            close(turn, (a.omegaRadPerSec() + b.omegaRadPerSec()) * span / 2, 1e-8, "Integrated timing interval heading");
        }
    }
    private static void run(String name, Runnable task) { task.run(); passed++; System.out.println("PASS " + name); }
    private static void rejects(Runnable task, String message) { try { task.run(); throw new AssertionError(message); } catch (IllegalArgumentException expected) { /* typed infeasibility */ } }
    private static void require(boolean condition, String message) { if (!condition) throw new AssertionError(message); }
    private static void close(double actual, double expected, double tolerance, String message) { require(Math.abs(actual - expected) <= tolerance, message + ": " + actual + " versus " + expected); }
}
