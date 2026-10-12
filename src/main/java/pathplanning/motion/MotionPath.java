package pathplanning.motion;

import java.util.List;
import pathplanning.geometric.Geometry.Pose;

/** Pure-Java implementation values for the optional offline motion planner, not a replacement World-State SPI. */
public final class MotionPath {
    private MotionPath() {}
    public record Bezier(Pose p0, Pose p1, Pose p2, Pose p3) {
        public Pose at(double u) {
            double v = 1-u;
            return new Pose(v*v*v*p0.x()+3*v*v*u*p1.x()+3*v*u*u*p2.x()+u*u*u*p3.x(),
                    v*v*v*p0.y()+3*v*v*u*p1.y()+3*v*u*u*p2.y()+u*u*u*p3.y(), 0);
        }
        public Pose derivative(double u) {
            double v=1-u;
            return new Pose(3*(v*v*(p1.x()-p0.x())+2*v*u*(p2.x()-p1.x())+u*u*(p3.x()-p2.x())),
                    3*(v*v*(p1.y()-p0.y())+2*v*u*(p2.y()-p1.y())+u*u*(p3.y()-p2.y())),0);
        }
        public Pose secondDerivative(double u) {
            return new Pose(6*((1-u)*(p2.x()-2*p1.x()+p0.x())+u*(p3.x()-2*p2.x()+p1.x())),
                    6*((1-u)*(p2.y()-2*p1.y()+p0.y())+u*(p3.y()-2*p2.y()+p1.y())),0);
        }
    }
    /** curvatureBound bounds the entire incoming interval; stop marks a genuine tangent discontinuity. */
    public record Knot(double distanceM, int segment, double u, double x, double y,
                       double tangentRadians, double curvatureBound, boolean stop) {}
    public record Curve(List<Bezier> segments, List<Knot> knots, double lengthM, String detail) {
        public Curve { segments=List.copyOf(segments); knots=List.copyOf(knots); }
    }
    public record Limits(double maxSpeed, double maxAcceleration, double maxCentripetalAcceleration,
                         double maxAngularSpeed, double maxAngularAcceleration, double startSpeed,
                         double endSpeed, double startHeading, double endHeading,
                         double startAngularSpeed, double endAngularSpeed) {
        public Limits(double maxSpeed,double maxAcceleration,double maxCentripetalAcceleration,
                      double maxAngularSpeed,double maxAngularAcceleration,double startSpeed,
                      double endSpeed,double startHeading,double endHeading) {
            this(maxSpeed,maxAcceleration,maxCentripetalAcceleration,maxAngularSpeed,maxAngularAcceleration,
                    startSpeed,endSpeed,startHeading,endHeading,0,0);
        }
    }
    public record Sample(double timeSec, double distanceM, int segment, double u,
                         double x, double y, double headingRadians, double tangentRadians,
                         double speedMps, double accelerationMps2, double vxMps, double vyMps,
                         double omegaRadPerSec) {}
    public record Profile(List<Sample> samples, double durationSec, double peakSpeedMps, String detail) {
        public Profile { samples=List.copyOf(samples); }
    }
}
