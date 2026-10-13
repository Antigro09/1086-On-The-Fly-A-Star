package pathplanning.motion;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import pathplanning.geometric.GeometricSolver;
import pathplanning.geometric.Geometry;
import pathplanning.geometric.Geometry.Envelope;
import pathplanning.geometric.Geometry.MapSnapshot;
import pathplanning.geometric.Geometry.Pose;
import pathplanning.geometric.Geometry.Rectangle;
import pathplanning.geometric.SweptCollision;
import pathplanning.motion.MotionPath.Bezier;
import pathplanning.motion.MotionPath.Curve;
import pathplanning.motion.MotionPath.Knot;

/** CPU-bounded mathematical/geometry checks, not hardware or drivetrain qualification. */
public final class BezierSuite {
    private static int checks;
    private static final Runnable POLL=()->{};
    private BezierSuite() {}
    public static void main(String[] args) {
        defaultRoute(); openCorner(); cornerObstacle(); edgeAndStatic(); stationaryAndDirections(); invalid(); cancelAndBounds(); streamingCandidates(); convertedGeometry(); exactControls();
        System.out.println("BezierSuite: "+checks+" checks passed");
    }
    private static MapSnapshot map(double width,double height,List<Envelope> obstacles) {
        double cell=.1;int x=(int)Math.ceil(width/cell),y=(int)Math.ceil(height/cell);
        return new MapSnapshot(width,height,cell,x,y,new boolean[x*y],obstacles);
    }
    private static Pose p(double x,double y){return new Pose(x,y,0);}
    private static Envelope obstacle(String id,double x0,double y0,double x1,double y1){return new Envelope(id,new Rectangle(x0,y0,x1,y1),0);}
    private static void require(boolean condition,String message){if(!condition)throw new AssertionError(message);checks++;}
    private static boolean curved(Curve curve){return curve.segments().stream().anyMatch(c->Math.abs(cross(c.p0(),c.p3(),c.p1()))>1e-9||Math.abs(cross(c.p0(),c.p3(),c.p2()))>1e-9);}
    private static double cross(Pose a,Pose b,Pose q){return(b.x()-a.x())*(q.y()-a.y())-(b.y()-a.y())*(q.x()-a.x());}
    private static double distance(Pose a,Pose b){return Math.hypot(a.x()-b.x(),a.y()-b.y());}
    private static void defaultRoute() {
        MapSnapshot m=map(8,4,List.of(obstacle("block",3,1.35,5,2.65)));
        Geometry.Input input=new Geometry.Input(p(.8,2),p(7.2,2),new Geometry.Footprint(.4,.4,.015),
                new Geometry.Constraints(2,2,3,0,0,0,0),m,new Geometry.Budget(500_000_000L,10000,30000,10000),true);
        Geometry.Output seed=new GeometricSolver().plan(input,()->false);
        require(seed.status()==Geometry.Status.SUCCESS,"representative geometric seed succeeds");
        List<Curve> candidates=BezierSmoother.candidates(seed.poses(),m,input.footprint().enclosingRadius(),null,POLL);
        require(candidates.stream().anyMatch(BezierSuite::curved),"default obstacle route produces genuinely noncollinear cubic geometry");
        for(Curve c:candidates)verify(c,m,input.footprint().enclosingRadius(),seed.poses().get(0),seed.poses().get(seed.poses().size()-1));
    }
    private static void openCorner() {
        MapSnapshot m=map(6,5,List.of());List<Pose> seed=List.of(p(1,1),p(4,1),p(4,4));
        List<Curve> curves=BezierSmoother.candidates(seed,m,.2,null,POLL);
        require(curves.size()>=2,"bounded multiple trim alternatives available");
        Curve smooth=curves.stream().filter(BezierSuite::curved).findFirst().orElseThrow();
        require(smooth.knots().stream().noneMatch(Knot::stop),"C1 rounded curve does not manufacture stops");
        for(int i=1;i<smooth.segments().size();i++) {
            Pose a=smooth.segments().get(i-1).derivative(1),b=smooth.segments().get(i).derivative(0);
            require(distance(a,b)<1e-9,"join derivative is C1, beyond only collinear directions");
        }
        for(Curve c:curves)verify(c,m,.2,seed.get(0),seed.get(2));
        List<Curve> slight=BezierSmoother.candidates(List.of(p(1,1),p(3,1),p(5,1+1e-8)),m,.2,null,POLL);
        require(slight.stream().filter(c->c.detail().startsWith("C1")).anyMatch(c->c.knots().stream().noneMatch(Knot::stop)),"tiny nonzero turn is rounded rather than hidden as a curvature-free tangent discontinuity");
        Curve fallback=curves.stream().filter(c->!curved(c)).findFirst().orElseThrow();
        require(fallback.knots().stream().anyMatch(Knot::stop),"straight corner fallback flags its actual tangent discontinuity");
    }
    private static void cornerObstacle() {
        List<Pose> seed=List.of(p(1,1),p(3,1),p(3,3));
        MapSnapshot m=map(5,5,List.of(obstacle("inside",2.2,1.12,2.85,1.7)));
        List<Curve> curves=BezierSmoother.candidates(seed,m,.08,null,POLL);
        require(!curves.isEmpty(),"corner-cut obstruction retains a smaller trim or stop fallback");
        for(Curve c:curves)verify(c,m,.08,seed.get(0),seed.get(2));
        MapSnapshot narrow=map(5,5,List.of(obstacle("tiny-inside",2.985,1.011,2.989,1.015)));
        List<Curve> tight=BezierSmoother.candidates(seed,narrow,.01,null,POLL);
        require(!tight.isEmpty(),"tight corner keeps valid fallback rather than unsafe curve");
        for(Curve c:tight)verify(c,narrow,.01,seed.get(0),seed.get(2));
        require(tight.size()<6,"collision certification rejects at least one trim near narrow obstacle");
        MapSnapshot blocked=map(5,5,List.of(obstacle("occupied",2.8,.5,3.2,3.5)));
        require(BezierSmoother.candidates(seed,blocked,.01,null,POLL).isEmpty(),"occupied seed fails instead of cutting obstacle");
    }
    private static void edgeAndStatic() {
        MapSnapshot m=map(4,4,List.of());
        List<Curve> edge=BezierSmoother.candidates(List.of(p(.2,.5),p(.2,3.5)),m,.2,null,POLL);
        require(!edge.isEmpty(),"exact straight footprint field edge remains safe");
        require(BezierSmoother.candidates(List.of(p(.19,.5),p(.19,3.5)),m,.2,null,POLL).isEmpty(),"field footprint boundary rejected");
        boolean[] cells=new boolean[1600];cells[20*40+20]=true;MapSnapshot stat=new MapSnapshot(4,4,.1,40,40,cells,List.of());
        require(BezierSmoother.candidates(List.of(p(1,2.05),p(3,2.05)),stat,.1,null,POLL).isEmpty(),"static occupancy respected");
        List<Pose> route=List.of(p(1,1),p(3,1),p(3,3));
        for(Curve c:BezierSmoother.candidates(route,stat,.1,null,POLL))verify(c,stat,.1,route.get(0),route.get(2));
    }
    private static void stationaryAndDirections() {
        MapSnapshot m=map(6,6,List.of());Pose a=new Pose(2,2,0),b=new Pose(2,2,Math.PI/2);
        List<Curve> stationary=BezierSmoother.candidates(List.of(a,b),m,.2,null,POLL);
        require(stationary.size()==1&&stationary.get(0).segments().isEmpty()&&stationary.get(0).lengthM()==0,"zero length and in-place heading stay stationary geometry");
        require(BezierSmoother.candidates(List.of(a,b),m,.2,0.0,POLL).isEmpty(),"stationary geometry cannot honor nonzero start motion tangent");
        List<Pose> strafe=List.of(new Pose(1,1,0),new Pose(1,4,0),new Pose(1,4,Math.PI));
        List<Curve> vertical=BezierSmoother.candidates(strafe,m,.2,Math.PI/2,POLL);
        require(!vertical.isEmpty(),"strafe with fixed independent robot heading supported");
        require(vertical.get(0).segments().get(vertical.get(0).segments().size()-1).p3().equals(strafe.get(strafe.size()-1)),"stationary endpoint duplicate preserves exact original final pose anchor");
        require(Math.abs(vertical.get(0).knots().get(0).tangentRadians()-Math.PI/2)<1e-8,"nonzero start tangent honored");
        List<Curve> reverse=BezierSmoother.candidates(List.of(p(4,2),p(1,2)),m,.2,Math.PI,POLL);
        require(!reverse.isEmpty(),"reverse travel independent of robot heading");
        require(BezierSmoother.candidates(List.of(p(1,1),p(4,1)),m,.2,Math.PI,POLL).isEmpty(),"incompatible start tangent rejected rather than silently reversed");
        List<Curve> initialCurve=BezierSmoother.candidates(List.of(p(1,1),p(4,2)),m,.2,Math.PI/2,POLL);
        require(initialCurve.stream().anyMatch(BezierSuite::curved),"start motion tangent may produce a verified curved approach");
        for(Curve c:initialCurve)verify(c,m,.2,p(1,1),p(4,2));
        List<Curve> uturn=BezierSmoother.candidates(List.of(p(1,2),p(4,2),p(1,2)),m,.2,null,POLL);
        require(!uturn.isEmpty()&&uturn.get(0).knots().stream().anyMatch(Knot::stop),"exact reversal uses genuine internal stop rather than a cusp");
    }
    private static void invalid() {
        MapSnapshot m=map(4,4,List.of());
        expectInvalid(()->BezierSmoother.candidates(List.of(p(Double.NaN,1)),m,.2,null,POLL));
        expectInvalid(()->BezierSmoother.candidates(List.of(p(1,1)),m,Double.POSITIVE_INFINITY,null,POLL));
        expectInvalid(()->BezierSmoother.candidates(List.of(p(1,1)),m,-.1,null,POLL));
        expectInvalid(()->BezierSmoother.candidates(List.of(p(1,1)),m,.2,Double.NaN,POLL));
        expectInvalid(()->BezierSmoother.candidates(List.of(p(-.1,1)),m,.2,null,POLL));
        expectInvalid(()->BezierSmoother.candidates(List.of(),m,.2,null,POLL));
        expectInvalid(()->BezierSmoother.candidates(List.of(p(1,1)),new MapSnapshot(4,4,.1,40,40,new boolean[1],List.of()),.2,null,POLL));
        expectInvalid(()->BezierSmoother.candidates(List.of(p(1,1)),map(4,4,List.of(obstacle("invalid",1,1,1,2))),.2,null,POLL));
        Bezier cusp=new Bezier(p(1,1),p(2,1),p(0,1),p(1,1));
        require(Double.isInfinite(BezierSmoother.curvatureBound(cusp)),"derivative hull containing origin rejects possible cusp until subdivision");
    }
    private static void expectInvalid(Runnable test){boolean invalid=false;try{test.run();}catch(IllegalArgumentException expected){invalid=true;}require(invalid,"invalid input rejected explicitly");}
    private static void cancelAndBounds() {
        MapSnapshot m=map(6,6,List.of());AtomicInteger calls=new AtomicInteger();RuntimeException marker=new RuntimeException("cancel-marker");boolean propagated=false;
        try{BezierSmoother.candidates(List.of(p(1,1),p(4,1),p(4,4)),m,.2,null,()->{if(calls.incrementAndGet()==20)throw marker;});}catch(RuntimeException cancelled){propagated=cancelled==marker;}
        require(propagated,"mid-certification cancellation propagates exact poll failure");
        List<Pose> huge=new ArrayList<>();for(int i=0;i<130;i++)huge.add(p(1+i*.02,1+(i%2)*.1));
        require(BezierSmoother.candidates(huge,m,.001,null,POLL).isEmpty(),"segment cap never drops input geometry");
        Curve c=BezierSmoother.candidates(List.of(p(1,1),p(4,1)),m,.1,null,POLL).get(0);boolean immutable=false;
        try{c.knots().clear();}catch(UnsupportedOperationException expected){immutable=true;}require(immutable,"candidate values immutable");
    }
    private static void streamingCandidates() {
        MapSnapshot m=map(6,5,List.of());List<Pose> seed=List.of(p(1,1),p(4,1),p(4,4));
        List<Curve> incumbents=new ArrayList<>();RuntimeException marker=new RuntimeException("refinement-deadline");boolean propagated=false;
        try {
            BezierSmoother.visitCandidates(seed,m,.2,null,()->{if(!incumbents.isEmpty())throw marker;},incumbents::add);
        } catch(RuntimeException stopped) { propagated=stopped==marker; }
        require(propagated&&incumbents.size()==1,"fully certified first candidate reaches callback before later refinement poll expires");
        MapSnapshot obstacle=map(5,5,List.of(obstacle("inside",2.2,1.12,2.85,1.7)));List<Curve> certified=new ArrayList<>();
        BezierSmoother.visitCandidates(List.of(p(1,1),p(3,1),p(3,3)),obstacle,.08,null,POLL,certified::add);
        require(!certified.isEmpty()&&certified.stream().noneMatch(c->c.detail().contains("trim=0.45")),"collision-uncertified large trim never reaches callback");
    }
    private static void convertedGeometry() {
        MapSnapshot map=map(6,5,List.of());List<Pose> near=List.of(p(.20001,.5),p(.20001,3),p(2.5,3));
        Curve original=BezierSmoother.candidates(near,map,.2,null,POLL).get(0);
        double originX=-314.125,originY=872.875;
        List<Bezier> converted=new ArrayList<>();
        for(Bezier b:original.segments())converted.add(new Bezier(roundtrip(b.p0(),originX,originY),roundtrip(b.p1(),originX,originY),roundtrip(b.p2(),originX,originY),roundtrip(b.p3(),originX,originY)));
        Curve actual=new Curve(converted,original.knots(),original.lengthM(),"translated control roundtrip");
        require(BezierSmoother.revalidate(actual,map,.2+1e-10,POLL),"nonzero-origin conversion near boundary revalidates actual controls with caller roundoff guard");
        MapSnapshot obstacle=map(6,5,List.of(obstacle("new-curve-interior",2.3,2,2.7,2.6)));
        Curve straight=BezierSmoother.candidates(List.of(p(1,1),p(4,1)),obstacle,.1,null,POLL).get(0);Bezier b=straight.segments().get(0);
        Curve perturbed=new Curve(List.of(new Bezier(b.p0(),p(2,4),b.p2(),b.p3())),straight.knots(),straight.lengthM(),"unsafe supplied control perturbation");
        require(!BezierSmoother.revalidate(perturbed,obstacle,.1,POLL),"unsafe conversion/control perturbation rejected even with unchanged safe endpoints and old knots");
        Curve stationary=BezierSmoother.candidates(List.of(p(2,2)),map,.2,null,POLL).get(0);
        require(BezierSmoother.revalidate(stationary,map,.2,POLL),"stationary one-knot geometry revalidates");
        RuntimeException marker=new RuntimeException("revalidation-cancel");boolean propagated=false;
        try{BezierSmoother.revalidate(actual,map,.2,()->{throw marker;});}catch(RuntimeException stopped){propagated=stopped==marker;}
        require(propagated,"revalidation preserves cancellation/budget poll exceptions");
    }
    private static Pose roundtrip(Pose p,double ox,double oy){return new Pose((p.x()+ox)-ox,(p.y()+oy)-oy,p.headingRadians());}
    private static void exactControls() {
        List<Bezier> controls=List.of(new Bezier(p(1,3),p(2,5),p(3,5),p(4,3)),
                new Bezier(p(4,3),p(5,1),p(6,1),p(7,3)),new Bezier(p(7,3),p(8,5),p(9,5),p(10,3)));
        MapSnapshot map=map(12,6,List.of(obstacle("first-broad-bend",2.1,2.7,2.9,3.3),
                obstacle("middle-broad-bend",5.1,2.7,5.9,3.3),obstacle("last-broad-bend",8.1,2.7,8.9,3.3)));
        Curve certified=BezierSmoother.certify(controls,map,.3,POLL);
        require(certified!=null&&certified.segments().equals(controls),"global broad S controls certified exactly without seed-corner trimming");
        require(certified.knots().stream().noneMatch(Knot::stop),"actual C1 broad spline joins do not force stops");
        require(certified.segments().get(0).at(.5).y()>4.4,"certified bend has substantial deviation across a whole segment");
        verify(certified,map,.3,controls.get(0).p0(),controls.get(2).p3());
        List<Bezier> unsafe=new ArrayList<>(controls);unsafe.set(0,new Bezier(p(1,3),p(2,3),p(3,3),p(4,3)));
        require(BezierSmoother.certify(unsafe,map,.3,POLL)==null,"unsafe global control polygon rejected by complete swept footprint certificate");
        Bezier horizontal=new Bezier(p(1,1),p(2,1),p(3,1),p(4,1));
        Bezier vertical=new Bezier(p(4,1),p(4,2),p(4,3),p(4,4));MapSnapshot open=map(6,5,List.of());
        Curve corner=BezierSmoother.certify(List.of(horizontal,vertical),open,.1,POLL);
        require(corner!=null&&corner.knots().stream().filter(Knot::stop).count()==1,"supplied true tangent discontinuity marked as an internal stop");
        require(BezierSmoother.certify(List.of(horizontal,new Bezier(p(4.01,1),p(4,2),p(4,3),p(4,4))),open,.1,POLL)==null,"disconnected supplied cubics rejected without implicit bridging");
        require(BezierSmoother.certify(List.of(new Bezier(p(1,1),p(Double.NaN,2),p(3,1),p(4,1))),open,.1,POLL)==null,"nonfinite supplied control rejected");
        require(BezierSmoother.certify(List.of(new Bezier(p(1,1),p(1,1),p(3,1),p(4,1))),open,.1,POLL)==null,"zero endpoint derivative rejected as nonregular");
        require(BezierSmoother.certify(List.of(new Bezier(p(2,2),p(3,2),p(1,2),p(2,2))),open,.1,POLL)==null,"interior cusp cannot receive a regular interval certificate");
        RuntimeException marker=new RuntimeException("certification-cancel");boolean propagated=false;
        try{BezierSmoother.certify(controls,map,.3,()->{throw marker;});}catch(RuntimeException cancelled){propagated=cancelled==marker;}
        require(propagated,"exact-control certification propagates cancellation unchanged");
        boolean immutable=false;try{certified.segments().clear();}catch(UnsupportedOperationException expected){immutable=true;}
        require(immutable,"exact-control output immutable");
    }
    private static void verify(Curve c,MapSnapshot map,double radius,Pose start,Pose end) {
        require(c.segments().size()<=128&&c.knots().size()<=1024,"certificate resources bounded");
        require(distance(c.segments().get(0).p0(),start)==0&&distance(c.segments().get(c.segments().size()-1).p3(),end)==0,"exact positions retained");
        double previousDistance=0;Knot previous=c.knots().get(0);
        for(int i=1;i<c.knots().size();i++) {
            Knot k=c.knots().get(i);require(k.distanceM()>previousDistance&&Double.isFinite(k.curvatureBound())&&k.curvatureBound()>=0,"strict arc knots and finite interval bound");
            Bezier segment=c.segments().get(k.segment());double from=previous.segment()==k.segment()?previous.u():0;
            for(int j=0;j<=8;j++) {
                double u=from+(k.u()-from)*j/8;Pose d=segment.derivative(u),a=segment.secondDerivative(u);double speed=Math.hypot(d.x(),d.y());
                double curvature=Math.abs(d.x()*a.y()-d.y()*a.x())/(speed*speed*speed);
                require(curvature<=k.curvatureBound()*(1+1e-9)+1e-8,"analytic incoming bound encloses dense curvature checks");
                Pose p=segment.at(u);require(SweptCollision.safe(map,p,p,radius,POLL),"independent dense occupancy regression check");
            }
            previous=k;previousDistance=k.distanceM();
        }
        require(Math.abs(c.lengthM()-previousDistance)<1e-12,"total numerical length matches final knot");
    }
}
