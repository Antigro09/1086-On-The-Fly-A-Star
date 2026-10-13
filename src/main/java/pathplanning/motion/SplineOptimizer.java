package pathplanning.motion;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Consumer;
import pathplanning.geometric.Geometry.MapSnapshot;
import pathplanning.geometric.Geometry.Pose;
import pathplanning.geometric.Geometry.Rectangle;
import pathplanning.motion.MotionPath.Bezier;
import pathplanning.motion.MotionPath.Curve;
import pathplanning.motion.MotionPath.Limits;

/** Bounded continuous-space refinement of a discrete route. All accepted controls are certified
 * independently; sampled obstacle potentials guide optimization but never establish safety.
 * Natural cubic spans share a global chord-length parameter and have continuous tangent/curvature.
 * Their anchors may move within a bounded neighborhood; they need not pass through grid corners. */
public final class SplineOptimizer {
    private static final int MAX_ANCHORS=48, MAX_EVALUATIONS=320, COST_SAMPLES=16, MAX_OBSTACLES=512;
    private static final double MAX_DRIFT=1.2;
    private SplineOptimizer() {}

    public static void visitCandidates(List<Pose> seed, MapSnapshot map, double radius,
            Double startTangent, Limits limits, Runnable poll, Consumer<Curve> accept) {
        Objects.requireNonNull(poll); Objects.requireNonNull(accept); Objects.requireNonNull(limits);
        poll.run();
        if (seed==null || seed.size()<2 || seed.size()>512 || map==null || !Double.isFinite(radius) || radius<0)
            return;
        List<Pose> unique=new ArrayList<>();
        for (Pose p:seed) {
            poll.run(); if (p==null || !Double.isFinite(p.x()) || !Double.isFinite(p.y())) return;
            if (unique.isEmpty() || distance(unique.get(unique.size()-1),p)>1e-8) unique.add(p);
        }
        if (unique.size()<2) return;
        unique.set(unique.size()-1,seed.get(seed.size()-1));
        List<Rectangle> obstacles=new ArrayList<>();
        for (var e:map.dynamic()) { poll.run(); if (obstacles.size()>=MAX_OBSTACLES)return;obstacles.add(e.bounds()); }
        if (map.hasStaticOccupancy()) for (int y=0;y<map.rows();y++) for (int x=0;x<map.columns();x++) {
            poll.run(); if (map.occupied(x,y)) {
                if (obstacles.size()>=MAX_OBSTACLES)return;
                obstacles.add(new Rectangle(x*map.cellSize(),y*map.cellSize(),Math.min(map.width(),(x+1)*map.cellSize()),Math.min(map.height(),(y+1)*map.cellSize())));
            }
        }
        Evaluator evaluator=new Evaluator(map,obstacles,radius,limits,poll);
        // Several anchor sets explore continuous routes, rather than preserving each grid turn.
        List<List<Pose>> anchorSets=List.of(coalesceTurns(simplify(unique,.35,poll)),simplify(unique,.9,poll),simplify(unique,.35,poll),unique);
        for (List<Pose> anchors:anchorSets) {
            if (evaluator.evaluations>=MAX_EVALUATIONS)break;
            if (anchors.size()>MAX_ANCHORS)continue;
            double[][] origin=coordinates(anchors),points=coordinates(anchors);
            Score best=evaluator.score(points,startTangent);
            publish(best,map,radius,poll,accept);
            for (double step:new double[]{.24,.12,.06,.03}) {
                for (int i=1;i<points.length-1 && evaluator.evaluations<MAX_EVALUATIONS;i++) {
                    poll.run(); double oldX=points[i][0],oldY=points[i][1],selectedX=oldX,selectedY=oldY;
                    Score selected=best;
                    for (int axis=0;axis<2 && evaluator.evaluations<MAX_EVALUATIONS;axis++) for (int sign:new int[]{-1,1}) {
                        if (evaluator.evaluations>=MAX_EVALUATIONS)break;
                        points[i][0]=oldX+(axis==0?sign*step:0);points[i][1]=oldY+(axis==1?sign*step:0);
                        if (Math.hypot(points[i][0]-origin[i][0],points[i][1]-origin[i][1])>MAX_DRIFT)continue;
                        Score candidate=evaluator.score(points,startTangent);
                        if (candidate.cost<selected.cost) { selected=candidate;selectedX=points[i][0];selectedY=points[i][1]; }
                    }
                    points[i][0]=selectedX;points[i][1]=selectedY;best=selected;
                }
                publish(best,map,radius,poll,accept);
                if (evaluator.evaluations>=MAX_EVALUATIONS)break;
            }
        }
    }

    private static void publish(Score score,MapSnapshot map,double radius,Runnable poll,Consumer<Curve> accept) {
        poll.run();
        if (score.controls==null || score.minimumClearance < 0)return;
        Curve certified=BezierSmoother.certify(score.controls,map,radius,poll);
        if (certified!=null) accept.accept(new Curve(certified.segments(),certified.knots(),certified.lengthM(),
                "Globally coupled cubic spline; continuous curvature; bounded anchor refinement in free space; "+certified.detail()));
    }

    private record Score(double cost,double minimumClearance,List<Bezier> controls) {}
    private static final class Evaluator {
        final MapSnapshot map;final List<Rectangle> obstacles;final double radius;final Limits limits;final Runnable poll;int evaluations;
        Evaluator(MapSnapshot map,List<Rectangle> obstacles,double radius,Limits limits,Runnable poll) {
            this.map=map;this.obstacles=obstacles;this.radius=radius;this.limits=limits;this.poll=poll;
        }
        Score score(double[][] points,Double tangent) {
            poll.run();evaluations++;
            List<Bezier> controls=fit(points,tangent);
            if (controls==null)return new Score(Double.POSITIVE_INFINITY,Double.NEGATIVE_INFINITY,null);
            double durationProxy=0,bending=0,collision=0,minimum=Double.POSITIVE_INFINITY;
            for (Bezier b:controls) {
                poll.run();
                for (int i=0;i<=COST_SAMPLES;i++) {
                    poll.run();double u=(double)i/COST_SAMPLES,weight=i==0||i==COST_SAMPLES?.5:1;
                    Pose p=b.at(u),v=b.derivative(u),a=b.secondDerivative(u);double speed=Math.hypot(v.x(),v.y());
                    if (!Double.isFinite(speed) || speed<1e-6)return new Score(Double.POSITIVE_INFINITY,Double.NEGATIVE_INFINITY,null);
                    double curvature=Math.abs(v.x()*a.y()-v.y()*a.x())/(speed*speed*speed),ds=weight*speed/COST_SAMPLES;
                    if (!Double.isFinite(curvature))return new Score(Double.POSITIVE_INFINITY,Double.NEGATIVE_INFINITY,null);
                    double cap=curvature>1e-9 ? Math.min(limits.maxSpeed(),Math.sqrt(Math.min(limits.maxAcceleration(),limits.maxCentripetalAcceleration())/curvature)) : limits.maxSpeed();
                    durationProxy+=ds/Math.max(.05,cap);bending+=curvature*curvature*ds;
                    double clearance=Math.min(Math.min(p.x(),map.width()-p.x()),Math.min(p.y(),map.height()-p.y()))-radius;
                    for (Rectangle r:obstacles) {
                        double dx=Math.max(Math.max(r.minX()-p.x(),0),p.x()-r.maxX()),dy=Math.max(Math.max(r.minY()-p.y(),0),p.y()-r.maxY());
                        clearance=Math.min(clearance,Math.hypot(dx,dy)-radius);
                    }
                    minimum=Math.min(minimum,clearance);
                    // A small comfort margin is an optimization preference, not additional physical inflation.
                    double penetration=Math.max(0,.035-clearance);
                    collision+=penetration*penetration*ds;
                }
            }
            double cost=durationProxy+.035*bending+5000*collision;
            return new Score(Double.isFinite(cost)?cost:Double.POSITIVE_INFINITY,minimum,controls);
        }
    }

    /** Natural cubic interpolation, clamped to the measured initial travel direction when supplied.
     * h is each chord length. Reusing its global parameter on both sides gives G2 curve joins;
     * local [0,1] Bézier derivatives differ by h but physical tangent and curvature agree. */
    static List<Bezier> fit(double[][] p,Double tangent) {
        int n=p.length;if(n<2||n>MAX_ANCHORS)return null;
        double[] h=new double[n-1];for(int i=0;i<n-1;i++) {
            h[i]=Math.hypot(p[i+1][0]-p[i][0],p[i+1][1]-p[i][1]);if(!Double.isFinite(h[i])||h[i]<.04)return null;
        }
        double[][] second=new double[2][n];
        for (int axis=0;axis<2;axis++) {
            double[] lower=new double[n],diag=new double[n],upper=new double[n],rhs=new double[n];
            diag[0]=1;diag[n-1]=1;
            if(tangent!=null) {
                diag[0]=2*h[0];upper[0]=h[0];
                rhs[0]=6*((p[1][axis]-p[0][axis])/h[0]-(axis==0?Math.cos(tangent):Math.sin(tangent)));
            }
            for(int i=1;i<n-1;i++) {
                lower[i]=h[i-1];diag[i]=2*(h[i-1]+h[i]);upper[i]=h[i];
                rhs[i]=6*((p[i+1][axis]-p[i][axis])/h[i]-(p[i][axis]-p[i-1][axis])/h[i-1]);
            }
            for(int i=1;i<n;i++) { double factor=lower[i]/diag[i-1];diag[i]-=factor*upper[i-1];rhs[i]-=factor*rhs[i-1]; }
            second[axis][n-1]=rhs[n-1]/diag[n-1];
            for(int i=n-2;i>=0;i--)second[axis][i]=(rhs[i]-upper[i]*second[axis][i+1])/diag[i];
        }
        List<Bezier> controls=new ArrayList<>();
        for(int i=0;i<n-1;i++) {
            double[] first=new double[2],last=new double[2];
            for(int axis=0;axis<2;axis++) {
                double secant=(p[i+1][axis]-p[i][axis])/h[i];
                first[axis]=p[i][axis]+h[i]/3*(secant-h[i]*(2*second[axis][i]+second[axis][i+1])/6);
                last[axis]=p[i+1][axis]-h[i]/3*(secant+h[i]*(second[axis][i]+2*second[axis][i+1])/6);
                if(!Double.isFinite(first[axis])||!Double.isFinite(last[axis]))return null;
            }
            controls.add(new Bezier(new Pose(p[i][0],p[i][1],0),new Pose(first[0],first[1],0),new Pose(last[0],last[1],0),new Pose(p[i+1][0],p[i+1][1],0)));
        }
        return List.copyOf(controls);
    }
    private static double[][] coordinates(List<Pose> points) { double[][] out=new double[points.size()][2];for(int i=0;i<out.length;i++){out[i][0]=points.get(i).x();out[i][1]=points.get(i).y();}return out; }
    /** Two consecutive same-direction turns often describe the entry/exit corners of one detour.
     * Replace them by one movable portal. This is a proposal, never a collision-safe shortcut;
     * the resulting whole spline must still pass the independent exact-controls certificate. */
    private static List<Pose> coalesceTurns(List<Pose> points) {
        List<Pose> out=new ArrayList<>();out.add(points.get(0));
        for(int i=1;i<points.size()-1;i++) {
            if(i+2<points.size()) {
                Pose before=points.get(i-1),a=points.get(i),b=points.get(i+1),after=points.get(i+2);
                double first=turn(before,a,b),second=turn(a,b,after);
                if(first*second>0 && Math.abs(first)>.2 && Math.abs(second)>.2 && Math.abs(first+second)>.6) {
                    out.add(new Pose((a.x()+b.x())/2,(a.y()+b.y())/2,0));i++;continue;
                }
            }
            out.add(points.get(i));
        }
        out.add(points.get(points.size()-1));return List.copyOf(out);
    }
    private static double turn(Pose a,Pose b,Pose c) { double x=b.x()-a.x(),y=b.y()-a.y(),u=c.x()-b.x(),v=c.y()-b.y();return Math.atan2(x*v-y*u,x*u+y*v); }
    private static List<Pose> simplify(List<Pose> p,double tolerance,Runnable poll) {
        if(p.size()<=2)return List.copyOf(p);poll.run();
        Pose a=p.get(0),b=p.get(p.size()-1);double dx=b.x()-a.x(),dy=b.y()-a.y(),square=dx*dx+dy*dy,max=0;int far=0;
        for(int i=1;i<p.size()-1;i++) { poll.run();Pose v=p.get(i);double t=square==0?0:Math.max(0,Math.min(1,((v.x()-a.x())*dx+(v.y()-a.y())*dy)/square));double d=Math.hypot(v.x()-a.x()-t*dx,v.y()-a.y()-t*dy);if(d>max){max=d;far=i;} }
        if(max<=tolerance)return List.of(a,b);
        List<Pose> left=simplify(p.subList(0,far+1),tolerance,poll),right=simplify(p.subList(far,p.size()),tolerance,poll);
        List<Pose> out=new ArrayList<>(left.subList(0,left.size()-1));out.addAll(right);return out;
    }
    private static double distance(Pose a,Pose b){return Math.hypot(b.x()-a.x(),b.y()-a.y());}
}
