package pathplanning.motion;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Objects;
import pathplanning.geometric.Geometry.Pose;
import pathplanning.motion.MotionPath.*;

/** Optional offline motion timing; no following, actuator or objective-completion authority.
 * maxAcceleration bounds the total linear acceleration norm. Curvature bounds apply to the
 * entire incoming knot interval, so normal and tangential acceleration are constrained together.
 * The reachable-speed passes minimize time in the fixed, conservative, piecewise-linear squared
 * speed envelope; they do not claim an optimal curve or a globally optimal robot trajectory.
 * Heading uses independent bounded angular trapezoids, preserving requested boundary angular
 * velocities. The shortest wrapped turn and its two adjacent full-turn alternatives are considered.
 * A zero linear endpoint permits waiting at that endpoint for angular synchronization. If both
 * linear endpoints move, unsupported longer synchronization is rejected without changing speeds.
 */
public final class TimeParameterizer {
    private static final int MAX_KNOTS = 16_384, MAX_NODES = 40_000, MAX_SEGMENTS = 4096;
    private static final Runnable NO_POLL = () -> {};
    private TimeParameterizer() {}

    /** Recompute cumulative arc coordinates using the same bounded integrator as the evaluator.
     * Keep this returned curve paired with its profile and serialize it for any external evaluator.
     * Source quadrature/chord estimates must not become a different physical speed coordinate. */
    public static Curve integratedCurve(Curve curve, Runnable poll) {
        Objects.requireNonNull(poll, "poll");
        poll.run();
        if (curve == null || curve.knots().isEmpty() || curve.knots().size() > MAX_KNOTS || curve.segments().size() > MAX_SEGMENTS)
            throw bad("Missing or excessive curve");
        if (curve.segments().isEmpty()) return curve;
        List<Knot> normalized = new ArrayList<>(); double distance = 0;
        Knot previous = curve.knots().get(0);
        if (previous == null || previous.segment() != 0 || previous.u() != 0 || previous.distanceM() != 0)
            throw bad("First knot must be the curve origin");
        normalized.add(previous);
        for (int i = 1; i < curve.knots().size(); i++) {
            poll.run(); Knot k = curve.knots().get(i);
            if (k == null || !finite(k.u()) || k.segment() < 0 || k.segment() >= curve.segments().size() ||
                    k.segment() < previous.segment() || k.segment() > previous.segment() + 1 || k.u() <= 0 || k.u() > 1 ||
                    (k.segment() == previous.segment() && k.u() <= previous.u()) ||
                    (k.segment() != previous.segment() && previous.u() != 1)) throw bad("Invalid arc-length knot sequence");
            double from = k.segment() == previous.segment() ? previous.u() : 0;
            distance += arcLength(curve.segments().get(k.segment()), from, k.u(), poll);
            if (!finite(distance) || distance > 100_000 || distance <= normalized.get(normalized.size() - 1).distanceM())
                throw bad("Invalid integrated curve length");
            normalized.add(new Knot(distance, k.segment(), k.u(), k.x(), k.y(), k.tangentRadians(), k.curvatureBound(), k.stop()));
            previous = k;
        }
        return new Curve(curve.segments(), normalized, distance, curve.detail() + "; timing arc coordinates recomputed by bounded integration");
    }

    /** Poll may throw the caller's cancellation/budget exception; it is deliberately propagated. */
    public static Profile profile(Curve curve, Limits limits, Runnable poll) {
        Objects.requireNonNull(poll, "poll");
        double[] curvatureBounds = validate(curve, limits, poll);
        double turn = wrap(limits.endHeading() - limits.startHeading());
        Angular angular = Angular.minimum(turn, limits.maxAngularSpeed(), limits.maxAngularAcceleration(), limits.startAngularSpeed(), limits.endAngularSpeed());
        if (curve.lengthM() == 0) {
            if (limits.startSpeed() != 0 || limits.endSpeed() != 0)
                throw bad("Stationary geometry cannot preserve a nonzero boundary speed");
            return assemble(curve, null, angular, angular.duration, limits.startHeading(), poll);
        }
        Grid grid = grid(curve, limits, curvatureBounds, poll);
        double[] speedSquared = grid.caps.clone();
        double startSquared = square(limits.startSpeed()), endSquared = square(limits.endSpeed());
        if (startSquared > grid.caps[0] + tolerance(grid.caps[0]) || endSquared > grid.caps[grid.caps.length - 1] + tolerance(grid.caps[grid.caps.length - 1]))
            throw bad("Boundary speed exceeds a conservative curvature/stop speed cap");
        speedSquared[0] = startSquared; speedSquared[speedSquared.length - 1] = endSquared;
        boolean converged = false;
        for (int pass = 0; pass < 8; pass++) {
            poll.run();
            for (int i = 1; i < speedSquared.length; i++) {
                poll.run();
                double reachable = reachable(speedSquared[i - 1], grid.distance[i] - grid.distance[i - 1], grid.curvature[i], limits.maxAcceleration());
                if (i == speedSquared.length - 1 && endSquared > reachable + tolerance(reachable))
                    throw bad("Insufficient distance to reach the requested end speed");
                speedSquared[i] = Math.min(speedSquared[i], reachable);
            }
            for (int i = speedSquared.length - 2; i >= 0; i--) {
                poll.run();
                double reachable = reachable(speedSquared[i + 1], grid.distance[i + 1] - grid.distance[i], grid.curvature[i + 1], limits.maxAcceleration());
                if (i == 0 && startSquared > reachable + tolerance(reachable))
                    throw bad("Insufficient distance to brake from the requested start speed");
                speedSquared[i] = Math.min(speedSquared[i], reachable);
            }
            if (feasible(grid, speedSquared, limits, poll)) { converged = true; break; }
        }
        if (!converged) throw bad("Reachable-speed passes did not certify the bounded envelope");
        // Never repair infeasible endpoints by silently lowering their requested velocities.
        if (Math.abs(speedSquared[0] - startSquared) > tolerance(startSquared) || Math.abs(speedSquared[speedSquared.length - 1] - endSquared) > tolerance(endSquared))
            throw bad("Boundary speeds cannot be preserved");
        speedSquared[0] = startSquared; speedSquared[speedSquared.length - 1] = endSquared;
        if (!feasible(grid, speedSquared, limits, poll)) throw bad("Exact boundary speeds cannot satisfy the conservative acceleration envelope");
        double[] times = new double[speedSquared.length];
        double[] speeds = new double[speedSquared.length];
        for (int i = 0; i < speeds.length; i++) speeds[i] = Math.sqrt(Math.max(0, speedSquared[i]));
        speeds[0] = limits.startSpeed(); speeds[speeds.length - 1] = limits.endSpeed();
        for (int i = 1; i < times.length; i++) {
            poll.run();
            double sum = speeds[i - 1] + speeds[i];
            if (!(sum > 0)) throw bad("Positive-distance interval has no reachable positive speed");
            times[i] = times[i - 1] + 2 * (grid.distance[i] - grid.distance[i - 1]) / sum;
            if (!Double.isFinite(times[i]) || !(times[i] > times[i - 1])) throw bad("Unrepresentable motion duration");
        }
        double translationDuration = times[times.length - 1];
        if (angular.duration > translationDuration) {
            if (limits.startSpeed() == 0 && limits.endSpeed() == 0) {
                double factor = angular.duration / translationDuration;
                for (int i = 0; i < times.length; i++) { times[i] *= factor; speeds[i] /= factor; }
                times[times.length - 1] = angular.duration;
            } else if (limits.endSpeed() == 0) {
                // Preserve the fastest translation and initial speed; hold at its stopped goal.
            } else if (limits.startSpeed() == 0) {
                // Wait at rest, then preserve the complete fastest translation and its final speed.
                double delay = angular.duration - translationDuration;
                for (int i = 0; i < times.length; i++) times[i] += delay;
                times[times.length - 1] = angular.duration;
            } else throw bad("Longer angular synchronization with two nonzero linear boundary speeds is unsupported; boundary velocities were not changed");
            translationDuration = angular.duration;
        }
        return assemble(curve, new Translation(grid.distance, times, speeds), angular,
                translationDuration, limits.startHeading(), poll);
    }

    /** Exact constant-acceleration/time interpolation followed by integrated Bézier arc inversion.
     * Finite times outside the profile are clamped to its endpoints. */
    public static Sample atTime(Curve curve, Profile profile, double timeSec) {
        Objects.requireNonNull(curve, "curve"); Objects.requireNonNull(profile, "profile");
        if (!Double.isFinite(timeSec) || !Double.isFinite(profile.durationSec()) || profile.durationSec() < 0 || profile.samples().isEmpty())
            throw bad("Invalid profile or sample time");
        double time = Math.max(0, Math.min(profile.durationSec(), timeSec));
        List<Sample> values = profile.samples();
        if (time == 0) return values.get(0);
        if (time == profile.durationSec()) return values.get(values.size() - 1);
        int lo = 0, hi = values.size() - 1;
        while (lo + 1 < hi) { int mid = (lo + hi) >>> 1; if (values.get(mid).timeSec() <= time) lo = mid; else hi = mid; }
        Sample a = values.get(lo), b = values.get(hi);
        double span = b.timeSec() - a.timeSec(), dt = time - a.timeSec();
        if (!(span > 0)) throw bad("Non-increasing timing samples");
        double acceleration = (b.speedMps() - a.speedMps()) / span;
        double angularAcceleration = (b.omegaRadPerSec() - a.omegaRadPerSec()) / span;
        double distance = a.distanceM() + a.speedMps() * dt + .5 * acceleration * dt * dt;
        double heading = a.headingRadians() + a.omegaRadPerSec() * dt + .5 * angularAcceleration * dt * dt;
        return sample(curve, time, Math.max(a.distanceM(), Math.min(b.distanceM(), distance)),
                Math.max(0, a.speedMps() + acceleration * dt), acceleration, wrap(heading),
                a.omegaRadPerSec() + angularAcceleration * dt, NO_POLL);
    }

    private static double[] validate(Curve curve, Limits l, Runnable poll) {
        if (curve == null || l == null || curve.knots().isEmpty() || curve.knots().size() > MAX_KNOTS || curve.segments().size() > MAX_SEGMENTS)
            throw bad("Missing or excessive curve");
        if (!finite(curve.lengthM()) || curve.lengthM() < 0 || curve.lengthM() > 100_000)
            throw bad("Invalid curve length");
        for (double v : new double[]{l.maxSpeed(), l.maxAcceleration(), l.maxCentripetalAcceleration(), l.maxAngularSpeed(), l.maxAngularAcceleration()})
            if (!finite(v) || v < 1e-9 || v > 1e6) throw bad("Motion limits must be finite and in [1e-9,1e6]");
        if (!finite(l.startSpeed(), l.endSpeed(), l.startHeading(), l.endHeading(), l.startAngularSpeed(), l.endAngularSpeed()) || l.startSpeed() < 0 || l.endSpeed() < 0 ||
                l.startSpeed() > l.maxSpeed() || l.endSpeed() > l.maxSpeed() || Math.abs(l.startHeading()) > 1e6 || Math.abs(l.endHeading()) > 1e6)
            throw bad("Invalid boundary speed or holonomic heading");
        if (Math.abs(l.startAngularSpeed()) > l.maxAngularSpeed() || Math.abs(l.endAngularSpeed()) > l.maxAngularSpeed())
            throw bad("Boundary angular speed exceeds its limit");
        for (Bezier b : curve.segments()) {
            poll.run();
            if (b == null) throw bad("Missing Bézier segment");
            for (Pose p : new Pose[]{b.p0(), b.p1(), b.p2(), b.p3()})
                if (p == null || !finite(p.x(), p.y(), p.headingRadians()) || Math.abs(p.x()) > 10_000 || Math.abs(p.y()) > 10_000) throw bad("Invalid Bézier control point");
        }
        double[] certifiedBounds = new double[curve.knots().size()];
        Knot previous = null; int index = 0;
        for (Knot k : curve.knots()) {
            poll.run();
            if (k == null || !finite(k.distanceM(), k.u(), k.x(), k.y(), k.tangentRadians(), k.curvatureBound()) ||
                    k.distanceM() < 0 || k.u() < 0 || k.u() > 1 || k.curvatureBound() < 0 || k.curvatureBound() > 1e6 ||
                    Math.abs(k.x()) > 10_000 || Math.abs(k.y()) > 10_000 || Math.abs(k.tangentRadians()) > 1e6)
                throw bad("Invalid arc-length knot");
            if (previous == null && (k.distanceM() != 0 || k.segment() != 0 || k.u() != 0)) throw bad("First knot must be the curve origin");
            if (curve.lengthM() != 0) {
                if (k.segment() < 0 || k.segment() >= curve.segments().size()) throw bad("Knot segment outside curve");
                Pose actual = curve.segments().get(k.segment()).at(k.u());
                if (Math.hypot(actual.x() - k.x(), actual.y() - k.y()) > 1e-6) throw bad("Knot position differs from its Bézier");
                if (previous != null) {
                    if (!(k.distanceM() > previous.distanceM()) || k.segment() < previous.segment() || k.segment() > previous.segment() + 1 ||
                            (k.segment() == previous.segment() && !(k.u() > previous.u())) ||
                            (k.segment() != previous.segment() && (previous.u() != 1 || k.u() <= 0)))
                        throw bad("Arc-length knots must advance within contiguous segments");
                    double from = k.segment() == previous.segment() ? previous.u() : 0;
                    Bezier incoming = curve.segments().get(k.segment());
                    if (k.segment() != previous.segment()) {
                        if (Math.hypot(incoming.p0().x() - previous.x(), incoming.p0().y() - previous.y()) > 1e-8)
                            throw bad("Bézier segments are not position-continuous");
                        Pose left = curve.segments().get(previous.segment()).derivative(1), right = incoming.derivative(0);
                        double turn = wrap(Math.atan2(right.y(), right.x()) - Math.atan2(left.y(), left.x()));
                        if (Math.abs(turn) > 1e-7 && !previous.stop()) throw bad("Tangent-discontinuous join lacks its required stop");
                    }
                    double certified = BezierSmoother.curvatureBound(restrict(incoming, from, k.u()));
                    if (!finite(certified) || certified > k.curvatureBound() + Math.max(1e-10, certified * 1e-7))
                        throw bad("Incoming curvature bound is smaller than its whole-interval certificate");
                    // Even accepted outward-rounding discrepancies must never loosen acceleration caps.
                    certifiedBounds[index] = Math.max(k.curvatureBound(), certified);
                    double measured = arcLength(incoming, from, k.u(), poll);
                    double declared = k.distanceM() - previous.distanceM();
                    if (measured <= 0 || Math.abs(measured - declared) > 2e-11 + 1e-10 * measured)
                        throw bad("Knot distance differs from integrated Bézier arc length; use integratedCurve before timing");
                }
            }
            previous = k; index++;
        }
        if (curve.lengthM() == 0) {
            if (!curve.segments().isEmpty() || curve.knots().size() != 1) throw bad("Stationary curve must have one knot and no segments");
        } else if (curve.segments().isEmpty() || previous.segment() != curve.segments().size() - 1 || previous.u() != 1 ||
                Math.abs(previous.distanceM() - curve.lengthM()) > 1e-9 * Math.max(1, curve.lengthM()))
            throw bad("Curve does not end at its declared length");
        return certifiedBounds;
    }

    private static Bezier restrict(Bezier b, double from, double to) {
        Bezier left = to == 1 ? b : split(b, to)[0];
        return from == 0 ? left : split(left, from / to)[1];
    }
    private static Bezier[] split(Bezier b, double u) {
        Pose a = lerp(b.p0(), b.p1(), u), c = lerp(b.p1(), b.p2(), u), d = lerp(b.p2(), b.p3(), u);
        Pose e = lerp(a, c, u), f = lerp(c, d, u), point = lerp(e, f, u);
        return new Bezier[]{new Bezier(b.p0(), a, e, point), new Bezier(point, f, d, b.p3())};
    }
    private static Pose lerp(Pose a, Pose b, double u) { return new Pose(a.x() + (b.x() - a.x()) * u, a.y() + (b.y() - a.y()) * u, 0); }

    private record Point(double distance, double curvature, boolean stop) {}
    private record Grid(double[] distance, double[] curvature, double[] caps) {}
    private static Grid grid(Curve curve, Limits l, double[] curvatureBounds, Runnable poll) {
        List<Point> values = new ArrayList<>(); values.add(new Point(0, 0, false));
        List<Knot> knots = curve.knots();
        // The smoother's outward-rounded certificate can assign an ulp-sized bound to a line.
        boolean flat = Arrays.stream(curvatureBounds).allMatch(k -> k <= 1e-10);
        List<Double> extra = new ArrayList<>();
        if (flat) {
            double began = 0, start = l.startSpeed();
            for (int i = 1; i < knots.size(); i++) if (knots.get(i).stop() || i == knots.size() - 1) {
                double ended = knots.get(i).distanceM(), end = i == knots.size() - 1 ? l.endSpeed() : 0;
                double length = ended - began, a = l.maxAcceleration(), cap = square(l.maxSpeed());
                double peak = Math.min(cap, a * length + (square(start) + square(end)) / 2);
                extra.add(began + (peak - square(start)) / (2 * a));
                extra.add(ended - (peak - square(end)) / (2 * a));
                began = ended; start = end;
            }
            extra.sort(Comparator.naturalOrder());
        }
        int extraAt = 0;
        for (int i = 1; i < knots.size(); i++) {
            poll.run(); Knot a = knots.get(i - 1), b = knots.get(i);
            List<Double> splits = new ArrayList<>(); splits.add((a.distanceM() + b.distanceM()) / 2);
            while (extraAt < extra.size() && extra.get(extraAt) <= b.distanceM()) {
                double d = extra.get(extraAt++); if (d > a.distanceM() && d < b.distanceM()) splits.add(d);
            }
            splits.sort(Comparator.naturalOrder());
            for (double d : splits) if (d > values.get(values.size() - 1).distance + 1e-12 && d < b.distanceM() - 1e-12)
                values.add(new Point(d, curvatureBounds[i], false));
            values.add(new Point(b.distanceM(), curvatureBounds[i], b.stop() && i < knots.size() - 1));
            if (values.size() > MAX_NODES) throw bad("Timing refinement exceeds node capacity");
        }
        double[] distance = new double[values.size()], curvature = new double[values.size()], caps = new double[values.size()];
        Arrays.fill(caps, square(l.maxSpeed()));
        for (int i = 0; i < values.size(); i++) {
            poll.run(); Point p = values.get(i); distance[i] = p.distance; curvature[i] = p.curvature;
            if (p.stop) caps[i] = 0;
            if (i > 0 && curvature[i] > 0) {
                double cap = Math.min(l.maxCentripetalAcceleration(), l.maxAcceleration()) / curvature[i];
                caps[i] = Math.min(caps[i], cap); caps[i - 1] = Math.min(caps[i - 1], cap);
            }
        }
        return new Grid(distance, curvature, caps);
    }

    /** Maximum reachable squared speed with constant tangential acceleration across this edge. */
    private static double reachable(double x, double distance, double curvature, double acceleration) {
        if (!(distance > 0)) throw bad("Timing nodes must have positive arc-length separation");
        if (curvature == 0) return x + 2 * distance * acceleration;
        double h = 2 * distance * curvature, h2 = h * h, r = 2 * distance * acceleration;
        double discriminant = r * r * (1 + h2) - h2 * x * x;
        if (discriminant < -tolerance(r * r * (1 + h2))) throw bad("Curvature consumes the total acceleration limit");
        return Math.min(acceleration / curvature, (x + Math.sqrt(Math.max(0, discriminant))) / (1 + h2));
    }
    private static boolean feasible(Grid g, double[] w, Limits l, Runnable poll) {
        for (int i = 1; i < w.length; i++) {
            poll.run();
            double tangent = (w[i] - w[i - 1]) / (2 * (g.distance[i] - g.distance[i - 1]));
            double normal = g.curvature[i] * Math.max(w[i], w[i - 1]);
            if (!finite(tangent, normal) || Math.hypot(tangent, normal) > l.maxAcceleration() + tolerance(l.maxAcceleration()) ||
                    normal > l.maxCentripetalAcceleration() + tolerance(l.maxCentripetalAcceleration())) return false;
        }
        return true;
    }

    private record Translation(double[] distances, double[] times, double[] speeds) {
        double[] at(double time) {
            int last = times.length - 1;
            if (time < times[0]) return new double[]{distances[0], speeds[0], 0};
            if (time >= times[last]) return new double[]{distances[last], speeds[last], 0};
            int lo = 0, hi = last;
            while (lo + 1 < hi) { int mid = (lo + hi) >>> 1; if (times[mid] <= time) lo = mid; else hi = mid; }
            double span = times[hi] - times[lo], dt = time - times[lo], a = (speeds[hi] - speeds[lo]) / span;
            return new double[]{distances[lo] + speeds[lo] * dt + .5 * a * dt * dt, speeds[lo] + a * dt, a};
        }
    }
    private record Angular(double angle, double sign, double initial, double terminal, double acceleration,
                           double peak, double accelerateTime, double cruiseTime, double decelerateTime, double duration) {
        static Angular minimum(double angle, double speed, double acceleration, double initial, double terminal) {
            Angular best = null;
            for (double target : new double[]{angle, angle + 2 * Math.PI, angle - 2 * Math.PI}) for (double sign : new double[]{1, -1}) {
                double d = sign * target, v0 = sign * initial, v1 = sign * terminal;
                if (d == 0 && v0 == v1) {
                    Angular zero = new Angular(target, sign, initial, terminal, acceleration, v0, 0, 0, 0, 0);
                    if (best == null || zero.duration < best.duration) best = zero;
                    continue;
                }
                double peakSquared = acceleration * d + (square(v0) + square(v1)) / 2;
                if (peakSquared < 0) continue;
                double peak = Math.min(speed, Math.sqrt(peakSquared));
                if (peak < Math.max(v0, v1) - tolerance(speed)) continue;
                peak = Math.max(peak, Math.max(v0, v1));
                if (!(peak > 0) || peak > speed + tolerance(speed)) continue;
                double accelerate = (peak - v0) / acceleration, decelerate = (peak - v1) / acceleration;
                double remaining = d - (2 * square(peak) - square(v0) - square(v1)) / (2 * acceleration);
                if (remaining < -tolerance(Math.max(Math.abs(d), (square(v0) + square(v1)) / acceleration))) continue;
                double cruise = Math.max(0, remaining / peak), duration = accelerate + cruise + decelerate;
                if (!finite(duration) || duration <= 0 || duration > 1e9) continue;
                if (decelerate > 0 && duration - decelerate == duration) continue;
                Angular candidate = new Angular(target, sign, initial, terminal, acceleration, peak, accelerate, cruise, decelerate, duration);
                if (best == null || duration < best.duration) best = candidate;
            }
            if (best == null) throw bad("No bounded angular profile preserves the requested boundary angular velocities");
            return best;
        }
        double factor(double total) { return initial == 0 && terminal == 0 && duration > 0 ? total / duration : 1; }
        double offset(double total) {
            if (total <= duration || initial == 0 && terminal == 0 || terminal == 0) return 0;
            if (initial == 0) return total - duration;
            throw bad("Longer angular synchronization with two nonzero angular boundary velocities is unsupported");
        }
        double[] at(double time, double total) {
            if (time <= 0) return new double[]{0, initial};
            if (time >= total) return new double[]{angle, terminal};
            if (duration == 0) {
                if (total > 0 && (initial != 0 || terminal != 0)) throw bad("Nonzero angular velocity cannot hold a zero-turn angular profile");
                return new double[]{0, initial};
            }
            double factor = factor(total), local = (time - offset(total)) / factor;
            if (local <= 0) return new double[]{0, initial};
            if (local >= duration) return new double[]{angle, terminal};
            double v0 = sign * initial, position, velocity;
            if (local <= accelerateTime) { position = v0 * local + .5 * acceleration * local * local; velocity = v0 + acceleration * local; }
            else if (local <= accelerateTime + cruiseTime) {
                position = v0 * accelerateTime + .5 * acceleration * accelerateTime * accelerateTime + peak * (local - accelerateTime); velocity = peak;
            } else {
                double remaining = duration - local, v1 = sign * terminal;
                position = sign * angle - v1 * remaining - .5 * acceleration * remaining * remaining; velocity = v1 + acceleration * remaining;
            }
            return new double[]{sign * position, sign * velocity / factor};
        }
        void boundaries(List<Double> result, double total, Runnable poll) {
            if (duration == 0) return;
            double factor = factor(total), offset = offset(total);
            double[] phase = {offset, offset + accelerateTime * factor, offset + (accelerateTime + cruiseTime) * factor, offset + duration * factor};
            if (initial == 0 && terminal == 0 || terminal != 0 && offset > 0) phase[3] = total;
            for (int i = 1; i < phase.length; i++) {
                poll.run(); double from = phase[i - 1], to = phase[i];
                if (to <= from) continue;
                double w0 = at(from, total)[1], w1 = at(to, total)[1];
                int steps = Math.max(1, (int)Math.ceil(Math.max(Math.abs(w0), Math.abs(w1)) * (to - from) / (Math.PI / 2)));
                if (steps > 1024) throw bad("Angular timing exceeds bounded winding/sample capacity");
                for (int step = 0; step <= steps; step++) { poll.run(); result.add(from + (to - from) * step / steps); }
            }
        }
    }
    private static Profile assemble(Curve curve, Translation translation, Angular angular, double duration, double heading, Runnable poll) {
        if (!finite(duration) || duration < 0 || duration > 1e9) throw bad("Unrepresentable profile duration");
        List<Double> boundaries = new ArrayList<>(); boundaries.add(0.0); boundaries.add(duration);
        if (translation != null) for (double t : translation.times) boundaries.add(t);
        angular.boundaries(boundaries, duration, poll);
        boundaries.sort(Comparator.naturalOrder());
        List<Sample> samples = new ArrayList<>(); double peak = 0, previous = -1;
        for (double time : boundaries) {
            poll.run(); time = Math.max(0, Math.min(duration, time)); if (time == previous) continue;
            if (previous >= 0 && time - previous <= 8 * Math.ulp(Math.max(time, previous)) && time != duration) continue;
            double[] linear = translation == null ? new double[]{0, 0, 0} : translation.at(time);
            double[] rotation = angular.at(time, duration);
            Sample value = sample(curve, time, linear[0], linear[1], linear[2], wrap(heading + rotation[0]), rotation[1], poll);
            samples.add(value); peak = Math.max(peak, value.speedMps()); previous = time;
        }
        return new Profile(samples, duration, peak,
                "Conservative fixed-envelope motion timing; total linear acceleration constrained; independent holonomic heading; not solver time or physical completion");
    }

    private static Sample sample(Curve curve, double time, double distance, double speed, double acceleration, double heading, double omega, Runnable poll) {
        Knot first = curve.knots().get(0);
        if (curve.lengthM() == 0) return new Sample(time, 0, 0, 0, first.x(), first.y(), heading, first.tangentRadians(), 0, 0, 0, 0, omega);
        double s = Math.max(0, Math.min(curve.lengthM(), distance));
        List<Knot> knots = curve.knots(); int lo = 0, hi = knots.size() - 1;
        while (lo + 1 < hi) { int mid = (lo + hi) >>> 1; if (knots.get(mid).distanceM() <= s) lo = mid; else hi = mid; }
        Knot a = knots.get(lo), b = knots.get(hi); int segment = b.segment();
        double from = a.segment() == segment ? a.u() : 0, to = b.u(), u;
        Bezier bezier = curve.segments().get(segment);
        if (s <= a.distanceM()) u = from;
        else if (s >= b.distanceM()) u = to;
        else {
            double span = arcLength(bezier, from, to, poll), target = span * (s - a.distanceM()) / (b.distanceM() - a.distanceM());
            double left = from, right = to;
            for (int j = 0; j < 35; j++) { poll.run(); double mid = (left + right) / 2; if (arcLength(bezier, from, mid, poll) < target) left = mid; else right = mid; }
            u = (left + right) / 2;
        }
        Pose p = bezier.at(u), derivative = bezier.derivative(u);
        double norm = Math.hypot(derivative.x(), derivative.y());
        double tangent = norm > 1e-12 ? Math.atan2(derivative.y(), derivative.x()) : s == 0 ? first.tangentRadians() : b.tangentRadians();
        Sample result = new Sample(time, s, segment, u, p.x(), p.y(), heading, tangent, speed, acceleration, speed * Math.cos(tangent), speed * Math.sin(tangent), omega);
        if (!finite(time, s, u, p.x(), p.y(), heading, tangent, speed, acceleration, result.vxMps(), result.vyMps(), omega)) throw bad("Nonfinite timed sample");
        return result;
    }

    // Adaptive Simpson integration is bounded; inverse evaluation never substitutes a polyline chord.
    private static double arcLength(Bezier b, double a, double z, Runnable poll) {
        if (a == z) return 0;
        double middle = (a + z) / 2, fa = derivativeNorm(b, a), fm = derivativeNorm(b, middle), fz = derivativeNorm(b, z);
        double whole = (z - a) * (fa + 4 * fm + fz) / 6;
        return integrate(b, a, z, fa, fm, fz, whole, 1e-11 * Math.max(1, whole), 12, poll);
    }
    private static double integrate(Bezier b, double a, double z, double fa, double fm, double fz, double whole, double error, int depth, Runnable poll) {
        poll.run(); double middle = (a + z) / 2, leftMid = (a + middle) / 2, rightMid = (middle + z) / 2;
        double fl = derivativeNorm(b, leftMid), fr = derivativeNorm(b, rightMid);
        double left = (middle - a) * (fa + 4 * fl + fm) / 6, right = (z - middle) * (fm + 4 * fr + fz) / 6;
        double difference = left + right - whole;
        if (Math.abs(difference) <= 15 * error) return left + right + difference / 15;
        if (depth == 0) throw bad("Bézier arc integration exceeded bounded refinement");
        return integrate(b, a, middle, fa, fl, fm, left, error / 2, depth - 1, poll) + integrate(b, middle, z, fm, fr, fz, right, error / 2, depth - 1, poll);
    }
    private static double derivativeNorm(Bezier b, double u) { Pose p = b.derivative(u); return Math.hypot(p.x(), p.y()); }
    private static double wrap(double value) { return Math.atan2(Math.sin(value), Math.cos(value)); }
    private static double square(double value) { return value * value; }
    private static double tolerance(double value) { return 1e-12 * Math.abs(value) + 32 * Math.ulp(Math.abs(value)); }
    private static boolean finite(double... values) { for (double v : values) if (!Double.isFinite(v)) return false; return true; }
    private static IllegalArgumentException bad(String detail) { return new IllegalArgumentException(detail); }
}
