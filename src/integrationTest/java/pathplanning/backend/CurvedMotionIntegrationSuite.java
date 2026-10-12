package pathplanning.backend;

import java.util.List;
import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import org.frcworldstate.core.Geometry.*;
import org.frcworldstate.core.PlannerBackend.*;
import pathplanning.motion.MotionPath;
import pathplanning.motion.TimeParameterizer;

/** Small CPU-only integration checks of the actual common planner; no robot or controller execution. */
public final class CurvedMotionIntegrationSuite {
    private static int passed;
    private CurvedMotionIntegrationSuite() {}
    public static void main(String[] args) {
        CurvedMotionPlanner planner=new CurvedMotionPlanner(System::nanoTime,()->1_000_000L);
        Request request=request("curves",0,0,0,0,0,30_000_000L,()->false);
        CurvedMotionPlanner.Proposal p=planner.plan(request,new CurvedMotionPlanner.MotionLimits(2,6,0));
        require(p.geometry().status()==Status.SUCCESS,"Default obstacle route gets a feasible motion profile: "+p.geometry().detail());
        require(p.geometry().trajectory()==null,"Owner SPI remains geometry-only");
        require(p.curve()!=null && p.profile()!=null && p.candidateCount()>1,"Candidates are compared by actual motion duration");
        require(p.curve().segments().stream().anyMatch(b->Math.abs((b.p1().x()-b.p0().x())*(b.p3().y()-b.p0().y())
                -(b.p1().y()-b.p0().y())*(b.p3().x()-b.p0().x()))>1e-6),"Selected route contains a genuine curved cubic");
        require(p.geometry().solverDurationNanos()==p.planningDurationNanos() && p.profile().durationSec()>0,"Solver duration and motion time are independent quantities");
        require(p.geometry().requestId().equals(request.requestId()) && p.geometry().snapshotId()==request.snapshotId(),"Request identity retained");
        require(p.plannedStartUs()==1_000_000L && Math.ceil(p.profile().durationSec()*1e6)<request.validUntilUs()-p.plannedStartUs(),"Motion fits remaining robot-clock horizon");
        MotionPath.Sample first=p.profile().samples().get(0),last=p.profile().samples().get(p.profile().samples().size()-1);
        require(first.speedMps()==0 && last.speedMps()==0,"Rest boundary velocities preserved");
        boolean accelerating=false,braking=false;
        for (int i=0;i<=120;i++) {
            MotionPath.Sample s=TimeParameterizer.atTime(p.curve(),p.profile(),p.profile().durationSec()*i/120);
            require(s.speedMps()<=2+1e-7,"Speed limit");
            if (!p.curve().segments().isEmpty()) {
                MotionPath.Bezier b=p.curve().segments().get(s.segment());
                var d=b.derivative(s.u());var dd=b.secondDerivative(s.u());double norm=Math.hypot(d.x(),d.y());
                double k=norm>1e-12?Math.abs(d.x()*dd.y()-d.y()*dd.x())/(norm*norm*norm):0;
                require(Math.hypot(s.accelerationMps2(),k*s.speedMps()*s.speedMps())<=2+1e-6,"Total acceleration envelope");
            }
            accelerating|=s.accelerationMps2()>.01;braking|=s.accelerationMps2()<-.01;
        }
        require(accelerating && braking,"Actual speed profile accelerates and anticipates braking");
        CurvedMotionPlanner.Proposal shifted=planner.plan(request("shifted",-7,11,0,0,0,30_000_000L,()->false),new CurvedMotionPlanner.MotionLimits(2,6,0));
        require(shifted.geometry().status()==Status.SUCCESS,"Nonzero field origin supports converted curves");
        require(Math.abs(shifted.curve().segments().get(0).p0().x()+6)<1e-9,"Converted exact start position");
        require(planner.plan(request("goal-speed",0,0,0,0,0,30_000_000L,()->false),new CurvedMotionPlanner.MotionLimits(2,6,.5)).geometry().status()==Status.INVALID_INPUT,
                "Goal velocity tolerance cannot be bypassed");
        AtomicInteger polls=new AtomicInteger();
        CurvedMotionPlanner.Proposal cancelled=planner.plan(request("cancel",0,0,0,0,0,30_000_000L,()->polls.incrementAndGet()>100),new CurvedMotionPlanner.MotionLimits(2,6,0));
        require(cancelled.geometry().status()==Status.CANCELLED && cancelled.curve()==null && cancelled.profile()==null,"Cancel removes all motion geometry");
        Request stale=request("short-horizon",0,0,0,0,0,1_001_000L,()->false);
        require(planner.plan(stale,new CurvedMotionPlanner.MotionLimits(2,6,0)).geometry().status()!=Status.SUCCESS,"Insufficient forecast horizon cannot yield a motion plan");
        CurvedMotionPlanner.Proposal prefix=planner.planRecedingHorizon(stale,new CurvedMotionPlanner.MotionLimits(2,6,0));
        require(prefix.geometry().status()==Status.SUCCESS && prefix.validatedPrefixSec()>0
                && prefix.validatedPrefixSec()<prefix.profile().durationSec(),"Rolling replanning qualifies only the forecast-covered prefix");
        CurvedMotionPlanner.Proposal spinning=planner.plan(request("spin",0,0,0,0,.8,30_000_000L,()->false),new CurvedMotionPlanner.MotionLimits(2,6,0));
        require(spinning.geometry().status()==Status.SUCCESS && Math.abs(spinning.profile().samples().get(0).omegaRadPerSec()-.8)<1e-9,
                "Replanning preserves the current angular velocity");
        movingGoal(planner);
        denseCourse(planner);
        System.out.println("PASS: "+passed+" common curved-motion integration checks; CPU-only, no hardware following");
    }
    private static void denseCourse(CurvedMotionPlanner planner) {
        List<Obstacle> obstacles=new ArrayList<>();long expiry=60_000_000L;
        for(int col=0;col<4;col++) {
            double x=2.6+2.5*col;
            for(int row=0;row<5;row++) obstacles.add(new Obstacle("barrier-"+col+"-"+row,
                    new Vec2(x,(col%2==0?.3:2.9)+.7*row),.38,.04,expiry,false));
        }
        double[][] scattered={{1.15,.8,.25},{1.15,5.3,.25},{4,.45,.18},{6.6,5.55,.18},{9,.45,.18},{11.45,5.3,.25}};
        for(int i=0;i<scattered.length;i++) { double[] v=scattered[i];obstacles.add(new Obstacle("scatter-"+i,new Vec2(v[0],v[1]),v[2],.03,expiry,false)); }
        Request r=new Request("dense-course","synthetic-motion",1,12,22,new FieldIdentity("offseason-synthetic","curve-gauntlet-12x6","1"),
                new Pose2(new Vec2(1,3),0),new Velocity2(new Vec2(0,0),0),new Goal(new Pose2(new Vec2(11.2,3),Math.PI/2),.05,.05,.05),
                new Footprint(.6,.6),new Constraints(3,3,3),new Bounds(0,0,12,6),obstacles,1_000_000,expiry,new SearchBudget(System.nanoTime(),2_000_000_000L),()->false);
        var p=planner.plan(r,new CurvedMotionPlanner.MotionLimits(2.4,6,0));
        require(p.geometry().status()==Status.SUCCESS,"Dense course produces a validated motion proposal");
        require(p.curve().detail().contains("Globally coupled"),"The chosen geometry uses whole-route optimization");
        require(p.curve().segments().size()<=12,"Route portals replace the 31 segment grid/corner pattern");
        require(p.profile().durationSec()<11,"Continuous route improves the old 13.39s corner-fillet profile under identical limits");
        double totalLength=0,straightLength=0,maxDeviation=0;
        for(int i=0;i<p.curve().segments().size();i++) {
            var b=p.curve().segments().get(i);double segmentLength=0,dev=0;
            for(var k:p.curve().knots())if(k.segment()==i && k.u()==1)segmentLength=k.distanceM()-totalLength;
            double dx=b.p3().x()-b.p0().x(),dy=b.p3().y()-b.p0().y(),chord=Math.hypot(dx,dy);
            for(int j=0;j<=64;j++) {var at=b.at(j/64.0);dev=Math.max(dev,Math.abs(dx*(at.y()-b.p0().y())-dy*(at.x()-b.p0().x()))/chord);}
            if(dev<=.02)straightLength+=segmentLength;totalLength+=segmentLength;maxDeviation=Math.max(maxDeviation,dev);
            if(i>0) require(Math.abs(signedCurvature(p.curve().segments().get(i-1),1)-signedCurvature(b,0))<1e-7,"Geometric curvature is continuous across entire spans");
        }
        require(straightLength/totalLength<.25,"Most route length is genuinely curved, not short fillets between grid chords");
        require(maxDeviation>.08,"Broad bends exceed the old 1-5cm local fillets");
        for(int i=0;i<=180;i++) {
            var s=TimeParameterizer.atTime(p.curve(),p.profile(),p.profile().durationSec()*i/180);
            double k=Math.abs(signedCurvature(p.curve().segments().get(s.segment()),s.u()));
            require(s.speedMps()<=3+1e-6 && Math.hypot(s.accelerationMps2(),k*s.speedMps()*s.speedMps())<=3+1e-6,"Dense course speed and total acceleration");
            double radius=r.footprint().boundingRadiusM()+.02;
            require(s.x()>=radius && s.x()<=12-radius && s.y()>=radius && s.y()<=6-radius,"Dense footprint remains within the field");
            for(var o:obstacles) {
                double dx=Math.max(0,Math.abs(s.x()-o.centerM().x())-o.envelopeRadiusM()),dy=Math.max(0,Math.abs(s.y()-o.centerM().y())-o.envelopeRadiusM());
                require(Math.hypot(dx,dy)>radius,"Independent dense regression scan respects each enclosing obstacle box");
            }
        }
    }
    private static double signedCurvature(MotionPath.Bezier b,double u) {var d=b.derivative(u);var a=b.secondDerivative(u);double n=Math.hypot(d.x(),d.y());return (d.x()*a.y()-d.y()*a.x())/(n*n*n);}
    private static void movingGoal(CurvedMotionPlanner planner) {
        CountDownLatch started=new CountDownLatch(1),release=new CountDownLatch(1);
        try (LatestRequestPlanner gate=new LatestRequestPlanner(r -> {
            if (r.requestId().equals("old-moving-goal")) {
                started.countDown(); try { if(!release.await(1,TimeUnit.SECONDS)) throw new AssertionError("Test release timeout"); }
                catch(InterruptedException interrupted) { Thread.currentThread().interrupt(); }
            }
            return planner.plan(r,new CurvedMotionPlanner.MotionLimits(2,6,0)).geometry();
        })) {
            gate.setEnabled(true);
            Request old=request("old-moving-goal",0,0,0,0,0,30_000_000L,()->false);
            gate.invalidate(LatestRequestPlanner.Context.from(old));
            var oldFuture=gate.submit(old);
            try { require(started.await(1,TimeUnit.SECONDS),"Older moving-goal work started"); }
            catch(InterruptedException interrupted) { throw new AssertionError(interrupted); }
            Request next=new Request("new-moving-goal",old.taskId(),old.epoch(),old.snapshotId(),old.obstacleMapVersion(),old.field(),old.startPose(),old.startVelocity(),
                    new Goal(new Pose2(new Vec2(6.5,1.7),Math.PI/2),.05,.05,.05),old.footprint(),old.constraints(),old.bounds(),old.obstacles(),
                    old.issuedUs(),old.validUntilUs(),new SearchBudget(System.nanoTime(),2_000_000_000L),()->false);
            var newFuture=gate.submit(next); release.countDown();
            try {
                require(newFuture.get(2,TimeUnit.SECONDS).status()==Status.SUCCESS,"Replacement goal gets a newly timed route");
                require(oldFuture.get(2,TimeUnit.SECONDS).status()!=Status.SUCCESS,"Older goal cannot regain success authority");
            } catch(Exception failure) { throw new AssertionError(failure); }
            require(gate.currentProposal(1_000_000L).orElseThrow().requestId().equals(next.requestId()),"Only latest moving goal can publish");
            gate.setEnabled(false);require(gate.currentProposal(1_000_000L).isEmpty(),"Disable removes moving-goal proposal authority");
        } finally { release.countDown(); }
    }
    private static Request request(String id,double dx,double dy,double vx,double vy,double omega,long expiry,Cancellation cancel) {
        return new Request(id,"synthetic-motion",1,12,22,new FieldIdentity("offseason-synthetic","motion-test","1"),
                new Pose2(new Vec2(1+dx,2+dy),0),new Velocity2(new Vec2(vx,vy),omega),
                new Goal(new Pose2(new Vec2(7+dx,2+dy),Math.PI/2),.05,.05,.05),new Footprint(.6,.6),new Constraints(2,2,3),
                new Bounds(dx,dy,8+dx,4+dy),List.of(new Obstacle("physical",new Vec2(4+dx,2+dy),.55,.05,expiry,false),
                    new Obstacle("swept",new Vec2(5.5+dx,3+dy),.25,.05,expiry,true)),
                1_000_000,expiry,new SearchBudget(System.nanoTime(),2_000_000_000L),cancel);
    }
    private static void require(boolean value,String message) { if(!value)throw new AssertionError(message);passed++; }
}
