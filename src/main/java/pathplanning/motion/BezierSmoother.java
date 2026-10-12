package pathplanning.motion;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Consumer;
import pathplanning.geometric.Geometry;
import pathplanning.geometric.Geometry.Envelope;
import pathplanning.geometric.Geometry.MapSnapshot;
import pathplanning.geometric.Geometry.Pose;
import pathplanning.geometric.Geometry.Rectangle;
import pathplanning.geometric.SweptCollision;
import pathplanning.motion.MotionPath.Bezier;
import pathplanning.motion.MotionPath.Curve;
import pathplanning.motion.MotionPath.Knot;

/** Bounded offline cubic candidates, with whole-interval disk collision and curvature certificates.
 * Arc lengths are numerical quadrature estimates, not an exact robot trajectory or drivetrain model. */
public final class BezierSmoother {
    private static final double[] TRIMS = {.45, .30, .18, .08, .025};
    private static final int MAX_SEGMENTS = 128, MAX_KNOTS = 1024, MAX_DEPTH = 16, MAX_NODES = 16384;
    private static final double[] GAUSS_X = {0, -.5384693101056831, .5384693101056831, -.9061798459386640, .9061798459386640};
    private static final double[] GAUSS_W = {.5688888888888889, .4786286704993665, .4786286704993665, .2369268850561891, .2369268850561891};
    private BezierSmoother() {}

    /** Radius is the already-computed raw robot disk plus clearance; add no second robot inflation.
     * poll must enforce the caller's monotonic budget/cancellation and its exceptions propagate. */
    public static List<Curve> candidates(List<Geometry.Pose> seed, Geometry.MapSnapshot map,
            double robotRadius, Double startTangentRadians, Runnable poll) {
        List<Curve> output=new ArrayList<>();
        visitCandidates(seed,map,robotRadius,startTangentRadians,poll,output::add);
        return List.copyOf(output);
    }
    /** Delivers each fully certified candidate immediately. This lets the caller profile an incumbent
     * before spending its remaining shared budget on later refinements. Consumer exceptions propagate. */
    public static void visitCandidates(List<Geometry.Pose> seed, Geometry.MapSnapshot map,
            double robotRadius, Double startTangentRadians, Runnable poll, Consumer<Curve> onCertified) {
        if(onCertified==null)throw new IllegalArgumentException("Missing certified candidate consumer");
        validate(seed, map, robotRadius, startTangentRadians, poll);
        List<Pose> points = new ArrayList<>();
        for (Pose p : seed) {
            poll.run();
            if (points.isEmpty() || !samePosition(points.get(points.size()-1), p)) points.add(p);
        }
        if(points.size()>1)points.set(points.size()-1,seed.get(seed.size()-1));
        if (!SweptCollision.safe(map, seed.get(0), seed.get(0), robotRadius, poll)) return;
        for (int i=1;i<points.size();i++) {
            poll.run();
            if (!SweptCollision.safe(map, points.get(i-1), points.get(i), robotRadius, poll)) return;
        }
        if (points.size()==1) {
            if (startTangentRadians!=null) return;
            Pose p=points.get(0);
            onCertified.accept(new Curve(List.of(), List.of(new Knot(0,0,0,p.x(),p.y(),0,0,false)),0,
                    "Stationary geometry; holonomic in-place rotation belongs to the separate profiler"));
            return;
        }
        Double normalizedTangent=startTangentRadians==null?null:wrap(startTangentRadians);
        Set<List<Bezier>> seen = new HashSet<>();
        for (double trim : TRIMS) {
            poll.run();
            Built built=build(points,trim,normalizedTangent);
            addCertified(onCertified,seen,built,map,robotRadius,poll,"C1 cubic corner candidate, trim="+trim);
        }
        // A piecewise straight fallback deliberately stops at its actual tangent discontinuities.
        // It is never advertised as a smooth passage through a corner or as globally unavoidable.
        if (normalizedTangent==null || aligned(normalizedTangent, direction(points.get(0),points.get(1)))) {
            addCertified(onCertified,seen,build(points,0,normalizedTangent),map,robotRadius,poll,
                    "Validated piecewise straight fallback; internal tangent discontinuities require stops");
        }
    }
    /** Rechecks the supplied controls, not earlier seed geometry or a cached certificate.
     * Useful after coordinate conversion. Caller may add a numerical conversion guard to radius;
     * this method adds no second robot footprint inflation and never changes the supplied profile. */
    public static boolean revalidate(Curve curve, MapSnapshot map, double radius, Runnable poll) {
        if(curve==null || curve.segments().size()>MAX_SEGMENTS || curve.knots().size()>MAX_KNOTS ||
                !finite(curve.lengthM()) || curve.lengthM()<0)return false;
        if(curve.segments().isEmpty()) {
            if(curve.lengthM()!=0 || curve.knots().size()!=1)return false;
            Knot knot=curve.knots().get(0);
            if(knot==null || knot.distanceM()!=0 || !finite(knot.x(),knot.y()))return false;
            Pose point=new Pose(knot.x(),knot.y(),0);
            validate(List.of(point),map,radius,null,poll);
            return SweptCollision.safe(map,point,point,radius,poll);
        }
        Pose previous=null;
        for(Bezier b:curve.segments()) {
            if(b==null)return false;
            for(Pose p:new Pose[]{b.p0(),b.p1(),b.p2(),b.p3()})
                if(p==null || !finite(p.x(),p.y(),p.headingRadians()) || Math.abs(p.x())>10000 || Math.abs(p.y())>10000)return false;
            if(previous!=null && !samePosition(previous,b.p0()))return false;
            previous=b.p3();
        }
        Pose start=curve.segments().get(0).p0(),end=curve.segments().get(curve.segments().size()-1).p3();
        validate(List.of(start,end),map,radius,null,poll);
        Certificate certificate=new Certificate(map,radius,poll);
        Pose derivative=curve.segments().get(0).derivative(0);
        if(!finite(norm(derivative)) || norm(derivative)==0)return false;
        certificate.knots.add(new Knot(0,0,0,start.x(),start.y(),directionVector(derivative),0,false));
        for(int i=0;i<curve.segments().size();i++) {
            poll.run();
            if(!certificate.interval(curve.segments().get(i),i,0,1,0))return false;
        }
        return true;
    }
    private static void validate(List<Pose> seed, MapSnapshot m, double radius, Double tangent, Runnable poll) {
        if(seed==null || seed.isEmpty() || seed.size()>4096 || m==null || poll==null || !finite(radius) || radius<0 || radius>1000 || tangent!=null&&!finite(tangent))
            throw new IllegalArgumentException("Invalid smoother input, footprint radius, tangent or seed size");
        if(!finite(m.width(),m.height(),m.cellSize()) || m.width()<=0 || m.height()<=0 || m.width()>1000 || m.height()>1000 || m.cellSize()<.001 || m.columns()<=0 || m.rows()<=0 ||
            (long)m.columns()*m.rows()>1_000_000 || m.occupancySize()!=(long)m.columns()*m.rows() || m.columns()!=Math.ceil(m.width()/m.cellSize()) || m.rows()!=Math.ceil(m.height()/m.cellSize()) || m.dynamic()==null || m.dynamic().size()>4096)
            throw new IllegalArgumentException("Invalid immutable occupancy snapshot");
        Set<String> ids=new HashSet<>();
        for(Envelope e:m.dynamic()) {
            poll.run();
            if(e==null || e.objectId()==null || e.objectId().isBlank() || !ids.add(e.objectId()) || e.bounds()==null || e.observationAgeMicros() < -1)
                throw new IllegalArgumentException("Invalid obstacle identity/age");
            Rectangle r=e.bounds();
            if(!finite(r.minX(),r.minY(),r.maxX(),r.maxY()) || Math.abs(r.minX())>10000 || Math.abs(r.minY())>10000 || Math.abs(r.maxX())>10000 || Math.abs(r.maxY())>10000 || r.minX()>=r.maxX() || r.minY()>=r.maxY())
                throw new IllegalArgumentException("Invalid raw obstacle envelope");
        }
        for(Pose p:seed) {
            poll.run();
            if(p==null || !finite(p.x(),p.y(),p.headingRadians()) || p.x()<0 || p.y()<0 || p.x()>m.width() || p.y()>m.height())
                throw new IllegalArgumentException("Invalid or out-of-bounds seed pose");
        }
    }
    private record Built(List<Bezier> segments, Set<Integer> stops) {}
    private static Built build(List<Pose> p,double trim,Double tangent) {
        int count=p.size();
        Pose[] entry=new Pose[count],exit=new Pose[count]; double[] trimLength=new double[count],handle=new double[count];
        boolean[] rounded=new boolean[count]; double[] edgeLength=new double[count-1],angle=new double[count-1];
        for(int i=0;i<count-1;i++){edgeLength[i]=distance(p.get(i),p.get(i+1));angle[i]=direction(p.get(i),p.get(i+1));}
        for(int i=0;i<count;i++) {entry[i]=p.get(i);exit[i]=p.get(i);}
        for(int i=1;i<count-1;i++) {
            double turn=Math.abs(wrap(angle[i]-angle[i-1]));
            if(trim>0 && turn>0 && turn<Math.PI-1e-5) {
                rounded[i]=true;trimLength[i]=trim*Math.min(edgeLength[i-1],edgeLength[i]);
                entry[i]=offset(p.get(i),angle[i-1],-trimLength[i]);exit[i]=offset(p.get(i),angle[i],trimLength[i]);
            }
        }
        double[] gap=new double[count-1];for(int i=0;i<count-1;i++)gap[i]=distance(exit[i],entry[i+1]);
        for(int i=1;i<count-1;i++) {
            double cap=.32*Math.min(gap[i-1],gap[i]);
            if(rounded[i]) {
                double turn=Math.abs(wrap(angle[i]-angle[i-1]));
                double circularHandle=(4.0/3)*trimLength[i]*Math.tan(turn/4)/Math.tan(turn/2);
                handle[i]=Math.min(cap,circularHandle);
            } else handle[i]=cap;
        }
        List<Bezier> segments=new ArrayList<>();Set<Integer> stops=new HashSet<>();
        for(int i=0;i<count-1;i++) {
            if(gap[i]<=0 || !finite(gap[i]))return null;
            double firstHandle=i==0 ? (tangent==null||aligned(tangent,angle[0])?gap[i]/3:Math.min(trim==0?1.0/3:trim,1.0/3)*gap[i]) : handle[i];
            double endHandle=i+1==count-1?gap[i]/3:handle[i+1];
            double firstAngle=i==0&&tangent!=null?tangent:angle[i];
            segments.add(new Bezier(exit[i],(i==0&&tangent!=null?offset(exit[i],firstAngle,firstHandle):along(exit[i],entry[i+1],firstHandle/gap[i])),along(entry[i+1],exit[i],endHandle/gap[i]),entry[i+1]));
            if(i+1<count-1) {
                if(rounded[i+1])segments.add(new Bezier(entry[i+1],offset(entry[i+1],angle[i],handle[i+1]),offset(exit[i+1],angle[i+1],-handle[i+1]),exit[i+1]));
                else if(Math.abs(wrap(angle[i+1]-angle[i]))>0)stops.add(segments.size()-1);
            }
            if(segments.size()>MAX_SEGMENTS)return null;
        }
        if(tangent!=null && !aligned(tangent,directionVector(segments.get(0).derivative(0))))return null;
        return new Built(List.copyOf(segments),Set.copyOf(stops));
    }
    private static void addCertified(Consumer<Curve> onCertified, Set<List<Bezier>> seen, Built built,
            MapSnapshot map,double radius,Runnable poll,String detail) {
        if(built==null || built.segments().isEmpty() || !seen.add(built.segments()))return;
        Certificate certificate=new Certificate(map,radius,poll);
        Bezier first=built.segments().get(0);Pose start=first.p0(),d=first.derivative(0);
        if(norm(d)==0 || !finite(norm(d)))return;
        certificate.knots.add(new Knot(0,0,0,start.x(),start.y(),directionVector(d),0,false));
        for(int i=0;i<built.segments().size();i++) {
            poll.run();
            if(!certificate.interval(built.segments().get(i),i,0,1,0))return;
            if(built.stops().contains(i)) {
                int last=certificate.knots.size()-1;Knot k=certificate.knots.get(last);
                certificate.knots.set(last,new Knot(k.distanceM(),k.segment(),k.u(),k.x(),k.y(),k.tangentRadians(),k.curvatureBound(),true));
            }
        }
        onCertified.accept(new Curve(built.segments(),certificate.knots,certificate.length,
                detail+"; numerical arc length error <= "+certificate.arcErrorBound+" m from summed chord/control-polygon brackets; conservative interval curvature and swept disk"));
    }
    private static final class Certificate {
        final MapSnapshot map;final double radius;final Runnable poll;final List<Knot> knots=new ArrayList<>();int nodes;double length,arcErrorBound;
        Certificate(MapSnapshot map,double radius,Runnable poll){this.map=map;this.radius=radius;this.poll=poll;}
        boolean interval(Bezier curve,int segment,double u0,double u1,int depth) {
            poll.run();if(++nodes>MAX_NODES || knots.size()>=MAX_KNOTS)return false;
            double chord=distance(curve.p0(),curve.p3()),polygon=distance(curve.p0(),curve.p1())+distance(curve.p1(),curve.p2())+distance(curve.p2(),curve.p3());
            double deviation=Math.max(pointSegmentDistance(curve.p1(),curve.p0(),curve.p3()),pointSegmentDistance(curve.p2(),curve.p0(),curve.p3()));
            // Distance to a convex chord capsule is convex. Its maximum over the control hull
            // is bounded by the control vertices; every point of the Bézier lies in that hull.
            double guardedDeviation=deviation==0?0:deviation+32*Math.ulp(Math.max(1,Math.max(norm(curve.p0()),norm(curve.p3()))));
            boolean collisionSafe=SweptCollision.safe(map,curve.p0(),curve.p3(),radius+guardedDeviation,poll);
            double bound=curvatureBound(curve),sampled=Math.max(pointCurvature(curve,0),Math.max(pointCurvature(curve,.5),pointCurvature(curve,1)));
            boolean tight=finite(bound,sampled) && bound<=Math.max(sampled*1.5,sampled+.02);
            boolean shortEnough=polygon<=.15 && polygon-chord<=1e-5;
            if(collisionSafe&&tight&&shortEnough) {
                double estimate=arcLength(curve),next=length+estimate;
                if(!finite(estimate,next) || estimate<=0 || next<=length)return false;
                length=next;arcErrorBound+=Math.max(0,polygon-chord)+32*Math.ulp(polygon);Pose end=curve.p3(),derivative=curve.derivative(1);
                if(norm(derivative)==0 || !finite(norm(derivative)))return false;
                knots.add(new Knot(length,segment,u1,end.x(),end.y(),directionVector(derivative),bound,false));return true;
            }
            if(depth>=MAX_DEPTH)return false;
            Bezier[] halves=split(curve);double mid=(u0+u1)/2;
            return interval(halves[0],segment,u0,mid,depth+1)&&interval(halves[1],segment,mid,u1,depth+1);
        }
    }
    /** Analytic conservative bound on the complete cubic interval, not sampled curvature. */
    static double curvatureBound(Bezier c) {
        Pose[] velocity={multiply(subtract(c.p1(),c.p0()),3),multiply(subtract(c.p2(),c.p1()),3),multiply(subtract(c.p3(),c.p2()),3)};
        Pose[] acceleration={c.secondDerivative(0),c.secondDerivative(1)};
        double maxSpeed=0,maxAcceleration=0,cross=0;
        for(Pose v:velocity){maxSpeed=Math.max(maxSpeed,norm(v));for(Pose a:acceleration)cross=Math.max(cross,Math.abs(v.x()*a.y()-v.y()*a.x()));}
        for(Pose a:acceleration)maxAcceleration=Math.max(maxAcceleration,norm(a));
        double lower=hullMinimumNorm(velocity)-64*Math.ulp(maxSpeed)-maxSpeed*1e-14;
        if(!finite(lower,cross,maxAcceleration) || lower<=0)return Double.POSITIVE_INFINITY;
        double upper=cross+16*Math.ulp(cross)+1e-14*maxSpeed*maxAcceleration;
        double result=upper/(lower*lower*lower);
        return finite(result)?result:Double.POSITIVE_INFINITY;
    }
    private static double hullMinimumNorm(Pose[] v) {
        double triangle=cross(v[0],v[1],v[2]);
        if(triangle!=0) {
            Pose zero=new Pose(0,0,0);double a=cross(v[0],v[1],zero),b=cross(v[1],v[2],zero),c=cross(v[2],v[0],zero);
            if(a>=0&&b>=0&&c>=0 || a<=0&&b<=0&&c<=0)return 0;
        }
        Pose zero=new Pose(0,0,0);
        return Math.min(pointSegmentDistance(zero,v[0],v[1]),Math.min(pointSegmentDistance(zero,v[1],v[2]),pointSegmentDistance(zero,v[2],v[0])));
    }
    private static double pointCurvature(Bezier c,double u) {
        Pose v=c.derivative(u),a=c.secondDerivative(u);double speed=norm(v);
        if(speed==0)return Double.POSITIVE_INFINITY;
        return Math.abs(v.x()*a.y()-v.y()*a.x())/(speed*speed*speed);
    }
    private static double arcLength(Bezier c) {
        double sum=0;for(int i=0;i<GAUSS_X.length;i++)sum+=GAUSS_W[i]*norm(c.derivative((GAUSS_X[i]+1)/2));
        double chord=distance(c.p0(),c.p3()),polygon=distance(c.p0(),c.p1())+distance(c.p1(),c.p2())+distance(c.p2(),c.p3());
        return Math.max(chord,Math.min(polygon,sum/2));
    }
    private static Bezier[] split(Bezier c) {
        Pose a=mid(c.p0(),c.p1()),b=mid(c.p1(),c.p2()),d=mid(c.p2(),c.p3()),e=mid(a,b),f=mid(b,d),g=mid(e,f);
        return new Bezier[]{new Bezier(c.p0(),a,e,g),new Bezier(g,f,d,c.p3())};
    }
    private static Pose along(Pose a,Pose b,double fraction){return new Pose(a.x()+(b.x()-a.x())*fraction,a.y()+(b.y()-a.y())*fraction,0);}
    private static Pose mid(Pose a,Pose b){return new Pose((a.x()+b.x())/2,(a.y()+b.y())/2,0);}
    private static Pose offset(Pose p,double direction,double distance){return new Pose(p.x()+Math.cos(direction)*distance,p.y()+Math.sin(direction)*distance,0);}
    private static Pose subtract(Pose a,Pose b){return new Pose(a.x()-b.x(),a.y()-b.y(),0);}
    private static Pose multiply(Pose p,double factor){return new Pose(p.x()*factor,p.y()*factor,0);}
    private static double cross(Pose a,Pose b,Pose p){return (b.x()-a.x())*(p.y()-a.y())-(b.y()-a.y())*(p.x()-a.x());}
    private static double pointSegmentDistance(Pose p,Pose a,Pose b) {
        double dx=b.x()-a.x(),dy=b.y()-a.y(),square=dx*dx+dy*dy;
        double t=square==0?0:Math.max(0,Math.min(1,((p.x()-a.x())*dx+(p.y()-a.y())*dy)/square));
        return Math.hypot(p.x()-a.x()-t*dx,p.y()-a.y()-t*dy);
    }
    private static double distance(Pose a,Pose b){return Math.hypot(b.x()-a.x(),b.y()-a.y());}
    private static double norm(Pose p){return Math.hypot(p.x(),p.y());}
    private static double direction(Pose a,Pose b){return Math.atan2(b.y()-a.y(),b.x()-a.x());}
    private static double directionVector(Pose p){return Math.atan2(p.y(),p.x());}
    private static boolean aligned(double a,double b){return Math.abs(wrap(a-b))<=1e-8;}
    private static double wrap(double angle){return Math.atan2(Math.sin(angle),Math.cos(angle));}
    private static boolean samePosition(Pose a,Pose b){return a.x()==b.x()&&a.y()==b.y();}
    private static boolean finite(double... values){for(double v:values)if(!Double.isFinite(v))return false;return true;}
}
