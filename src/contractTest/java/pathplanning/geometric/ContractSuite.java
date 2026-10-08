package pathplanning.geometric;

import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.*;
import org.frcworldstate.core.PlannerBackend;
import org.frcworldstate.core.Geometry.*;
import pathplanning.backend.AStarPlannerBackend;
import pathplanning.backend.LatestRequestPlanner;
import pathplanning.geometric.Geometry.*;

/** Dependency-free deterministic acceptance suite. Synthetic tests do not qualify robot hardware. */
public final class ContractSuite {
    private static int passed;
    private static final Budget BUDGET=new Budget(1_000_000_000L,100_000,200_000,250_000);
    public static void main(String[] args) throws Exception {
        run("static obstacle and segment-safe simplification",ContractSuite::staticObstacle);
        run("conservative dynamic swept envelopes",ContractSuite::dynamicObstacle);
        run("narrow passage accepted/rejected by footprint",ContractSuite::narrowPassages);
        run("invalid/NaN/infinite input and map rejection",ContractSuite::invalidInputs);
        run("occupied/out-of-bounds start and goal",ContractSuite::unsafeEndpoints);
        run("holonomic strafe/reverse, heading separate from travel",ContractSuite::holonomic);
        run("stationary and in-place rotation paths",ContractSuite::stationary);
        run("start/end speed constraints remain geometry metadata",ContractSuite::endpointSpeeds);
        run("diagonal corner cutting and thin obstacle sweep",ContractSuite::sweep);
        run("field edges and entire goal tolerance region",ContractSuite::edgesAndTolerance);
        run("immutable static/dynamic snapshot",ContractSuite::immutableSnapshot);
        run("no path and bounded expansion/queues",ContractSuite::noPathAndLimits);
        run("monotonic timeout and cancellation polling",ContractSuite::timeoutCancellation);
        run("pinned adapter identities, clocks, geometry-only output",ContractSuite::adapter);
        run("adapter velocities, obsolete/invalid budgets and cancellation",ContractSuite::adapterRejects);
        run("older solver cannot publish after newer request",ContractSuite::latestWins);
        run("disable/cancel/snapshot/pose/source invalidation",ContractSuite::invalidation);
        System.out.println("PASS: "+passed+" acceptance groups; CPU synthetic only; no hardware execution");
    }
    private interface Checked { void run() throws Exception; }
    private static void run(String name,Checked test) throws Exception { test.run(); passed++; System.out.println("PASS "+name); }
    private static void require(boolean ok,String why) { if(!ok) throw new AssertionError(why); }
    private static void throwsIllegal(Runnable r) { try { r.run();throw new AssertionError("Expected invalid input"); } catch(IllegalArgumentException expected) {} }
    private static MapSnapshot map(List<Envelope> obstacles) { return new MapSnapshot(6,4,0.1,60,40,new boolean[2400],obstacles); }
    private static Footprint footprint() { return new Footprint(0.4,0.4,0); }
    private static Constraints constraints() { return new Constraints(3,3,4,0,0,0,0.1); }
    private static Input input(MapSnapshot map) { return new Input(new Pose(0.6,2,0),new Pose(5.4,2,Math.PI/2),footprint(),constraints(),map,BUDGET,true); }
    private static Input endpoints(Input i,Pose start,Pose goal) { return new Input(start,goal,i.footprint(),i.constraints(),i.map(),i.budget(),i.simplify()); }
    private static Output plan(Input i) { return new GeometricSolver().plan(i,()->false); }
    private static void success(Input i,Output result) {
        require(result.status()==Status.SUCCESS,result.status()+": "+result.detail());
        require(result.poses().get(0).equals(i.start()),"Start pose lost");
        require(result.poses().get(result.poses().size()-1).equals(i.goal()),"Goal pose lost");
        require(result.planningDurationNanos()>=0,"Negative solver duration");
        require(GeometricSolver.validatePath(i.map(),result.poses(),i.footprint().enclosingRadius(),()->{}),"Path fails swept footprint check");
    }
    private static Envelope block(String id,double minX,double minY,double maxX,double maxY) { return new Envelope(id,new Rectangle(minX,minY,maxX,maxY),100_000); }
    private static void staticObstacle() {
        boolean[] grid=new boolean[2400];
        for(int y=10;y<30;y++) for(int x=25;x<35;x++) grid[y*60+x]=true;
        Input i=input(new MapSnapshot(6,4,0.1,60,40,grid,List.of())); Output out=plan(i); success(i,out);
        require(out.poses().size()>3,"Straight shortcut illegally crossed wall");
        Input raw=new Input(i.start(),i.goal(),i.footprint(),i.constraints(),i.map(),i.budget(),false);
        Output unsimplified=plan(raw); success(raw,unsimplified);
        require(out.poses().size()<=unsimplified.poses().size(),"Simplification grows route");
    }
    private static void dynamicObstacle() {
        Input i=input(map(List.of(block("moving-whole-horizon",2,0.8,4,3.1)))); Output out=plan(i);success(i,out);
        require(out.poses().stream().anyMatch(p->p.y()<0.8||p.y()>3.1),"Swept forecast ignored");
        // A moving object sweeps the entire field width: no schedule is invented to slip through later.
        require(plan(input(map(List.of(block("swept-wall",2.9,0,3.1,4))))).status()==Status.NO_PATH,"Unbounded timed crossing claimed");
    }
    private static void narrowPassages() {
        Input roomy=input(map(List.of(block("low",2,0,4,1.55),block("high",2,2.45,4,4)))); success(roomy,plan(roomy));
        Input tight=input(map(List.of(block("low",2,0,4,1.8),block("high",2,2.2,4,4))));
        require(plan(tight).status()==Status.NO_PATH,"Robot footprint squeezed through insufficient gap");
    }
    private static void invalidInputs() {
        Input base=input(map(List.of()));
        for(double value:new double[]{Double.NaN,Double.POSITIVE_INFINITY,Double.NEGATIVE_INFINITY})
            require(plan(endpoints(base,new Pose(value,2,0),base.goal())).status()==Status.INVALID_INPUT,"Nonfinite pose accepted");
        require(plan(new Input(base.start(),base.goal(),new Footprint(-1,1,0),constraints(),base.map(),BUDGET,true)).status()==Status.INVALID_INPUT,"Bad footprint");
        require(plan(new Input(base.start(),base.goal(),footprint(),new Constraints(0,1,1,0,0,0,0),base.map(),BUDGET,true)).status()==Status.INVALID_INPUT,"Bad constraints");
        require(plan(input(new MapSnapshot(6,4,0.1,60,40,new boolean[1],List.of()))).status()==Status.INVALID_INPUT,"Ragged occupancy accepted");
        require(plan(input(map(List.of(block("same",2,1,3,2),block("same",4,1,5,2))))).status()==Status.INVALID_INPUT,"Duplicate obstacle ID accepted");
        require(plan(input(map(List.of(block("overflow",-1e308,2.1,1e308,2.2))))).status()==Status.INVALID_INPUT,"Uncomputable envelope accepted");
        require(plan(null).status()==Status.INVALID_INPUT,"Null input accepted");
        throwsIllegal(()->new Vec2(Double.NaN,0)); throwsIllegal(()->new PlannerBackend.Footprint(0,1));
    }
    private static void unsafeEndpoints() {
        Input base=input(map(List.of(block("occupied",0.3,1.7,0.9,2.3)))); require(plan(base).status()==Status.UNSAFE_START,"Occupied start escaped");
        base=input(map(List.of(block("occupied",5.1,1.7,5.7,2.3)))); require(plan(base).status()==Status.UNSAFE_GOAL,"Occupied goal accepted");
        base=input(map(List.of())); require(plan(endpoints(base,new Pose(-0.1,2,0),base.goal())).status()==Status.UNSAFE_START,"Start clamped");
        require(plan(endpoints(base,base.start(),new Pose(6.1,2,0))).status()==Status.UNSAFE_GOAL,"Goal clamped");
    }
    private static void holonomic() {
        Input base=input(map(List.of()));
        for(Pose goal:List.of(new Pose(0.6,3,0),new Pose(0.4,2,0),new Pose(5,2,Math.PI))) {
            Input i=endpoints(base,new Pose(1,2,0),goal);Output out=plan(i);success(i,out);
            if(goal.headingRadians()==Math.PI) require(out.poses().get(out.poses().size()-2).headingRadians()==0,"Heading replaced with travel direction");
        }
    }
    private static void stationary() {
        Input base=input(map(List.of())); Input still=endpoints(base,base.start(),base.start());success(still,plan(still));
        Input rotation=endpoints(base,new Pose(2,2,0),new Pose(2,2,Math.PI/2));Output out=plan(rotation);success(rotation,out);
        require(out.poses().stream().allMatch(p->p.x()==2&&p.y()==2),"In-place rotation translated");
        // Center clear but a rectangle corner sweeps the obstacle at some heading: enclosing disk rejects it.
        Input blocked=endpoints(input(map(List.of(block("rotation-corner",2.23,1.9,2.4,2.1)))),rotation.start(),rotation.goal());
        require(plan(blocked).status()==Status.UNSAFE_START,"Unsafe rectangle rotation admitted by center-only test");
    }
    private static void endpointSpeeds() {
        Input base=input(map(List.of()));
        Input valid=new Input(base.start(),base.goal(),footprint(),new Constraints(3,3,4,1,2,0,0.1),base.map(),BUDGET,true);
        success(valid,plan(valid));
        for(Constraints c:List.of(new Constraints(3,3,4,4,0,0,0.1),new Constraints(3,3,4,0,4,0,0.1),new Constraints(3,3,4,0,Double.NaN,0,0.1)))
            require(plan(new Input(base.start(),base.goal(),footprint(),c,base.map(),BUDGET,true)).status()==Status.INVALID_INPUT,"Endpoint speed constraint ignored");
    }
    private static void sweep() {
        boolean[] cells=new boolean[4];cells[1]=true;cells[2]=true;
        MapSnapshot grid=new MapSnapshot(2,2,1,2,2,cells,List.of());
        require(!SweptCollision.safe(grid,new Pose(0.5,0.5,0),new Pose(1.5,1.5,0),0.05,()->{}),"Diagonal squeezed between touching cells");
        require(SweptCollision.intersects(new Pose(0,0,0),new Pose(10,0,0),new Rectangle(4.999,-0.001,5.001,0.001),0.001),"Thin obstacle missed between samples");
        List<Pose> unsafe=List.of(new Pose(0.5,0.5,0),new Pose(1.5,1.5,0));
        require(!GeometricSolver.validatePath(grid,unsafe,0.05,()->{}),"Invalid shortcut validated");
        boolean[] contact=new boolean[16];contact[8]=true;
        MapSnapshot tangent=new MapSnapshot(4,4,1,4,4,contact,List.of());
        require(!SweptCollision.safe(tangent,new Pose(1.5,2.5,0),new Pose(1.5,2.5,0),0.5,()->{}),"Lower-x grid tangency skipped");
        contact=new boolean[16];contact[2]=true;tangent=new MapSnapshot(4,4,1,4,4,contact,List.of());
        require(!SweptCollision.safe(tangent,new Pose(2.5,1.5,0),new Pose(2.5,1.5,0),0.5,()->{}),"Lower-y grid tangency skipped");
        require(SweptCollision.intersects(new Pose(5,4.5,0),new Pose(5,4.5,0),new Rectangle(-1e308,5,1e308,6),0.6),"Overflow treated as clearance");
        contact=new boolean[2400];contact[20*60+16]=true;tangent=new MapSnapshot(6,4,0.1,60,40,contact,List.of());
        double boundary=17*0.1;
        require(!SweptCollision.safe(tangent,new Pose(boundary+0.5,2.05,0),new Pose(boundary+0.5,2.05,0),0.5,()->{}),"Decimal grid boundary tangency skipped");
    }
    private static void edgesAndTolerance() {
        Input base=input(map(List.of())); require(plan(endpoints(base,new Pose(0.01,2,0),base.goal())).status()==Status.UNSAFE_START,"Field edge footprint ignored");
        Input i=new Input(base.start(),new Pose(5.65,2,0),footprint(),new Constraints(3,3,4,0,0,0.2,0.1),base.map(),BUDGET,true);
        require(plan(i).status()==Status.UNSAFE_GOAL,"Tolerance region extends beyond field");
    }
    private static void immutableSnapshot() {
        boolean[] cells=new boolean[2400]; cells[20*60+30]=true;
        List<Envelope> obstacles=new ArrayList<>(List.of(block("retained",2.5,1,3.5,3)));
        MapSnapshot frozen=new MapSnapshot(6,4,0.1,60,40,cells,obstacles); Arrays.fill(cells,false);obstacles.clear();
        require(frozen.occupied(30,20)&&frozen.dynamic().size()==1,"Snapshot aliased frame/grid mutation");
        Input i=input(frozen);success(i,plan(i));
    }
    private static void noPathAndLimits() {
        Input wall=input(map(List.of(block("wall",2.9,0,3.1,4)))); require(plan(wall).status()==Status.NO_PATH,"Wall route returned");
        Input bounded=new Input(wall.start(),wall.goal(),wall.footprint(),wall.constraints(),wall.map(),new Budget(1_000_000_000L,1,200_000,250_000),true);
        require(plan(bounded).status()==Status.RESOURCE_LIMIT,"Expansion cap ignored");
        bounded=new Input(wall.start(),wall.goal(),wall.footprint(),wall.constraints(),wall.map(),new Budget(1_000_000_000L,100_000,1,250_000),true);
        require(plan(bounded).status()==Status.RESOURCE_LIMIT,"Queue cap ignored");
    }
    private static void timeoutCancellation() {
        AtomicLong clock=new AtomicLong();Input base=input(map(List.of()));
        Input tiny=new Input(base.start(),base.goal(),base.footprint(),base.constraints(),base.map(),new Budget(10,100,100,250_000),true);
        Output out=new GeometricSolver(()->clock.getAndAdd(5)).plan(tiny,()->false);require(out.status()==Status.TIMEOUT,"Monotonic timeout ignored");
        require(new GeometricSolver().plan(base,()->true).status()==Status.CANCELLED,"Pre-cancellation ignored");
        AtomicInteger polls=new AtomicInteger();Input wall=input(map(List.of(block("wall",2.9,0,3.1,4))));
        require(new GeometricSolver().plan(wall,()->polls.incrementAndGet()>30).status()==Status.CANCELLED,"Cancellation not polled during swept scans");
    }
    private static final FieldIdentity FIELD=new FieldIdentity("offseason","synthetic-map","r1");
    private static PlannerBackend.Request request(String id,long snapshot,long map,long epoch,PlannerBackend.Cancellation cancellation) {
        return new PlannerBackend.Request(id,"task",epoch,snapshot,map,FIELD,new Pose2(new Vec2(0.6,2),0),Velocity2.zero(),
            new PlannerBackend.Goal(new Pose2(new Vec2(5.4,2),Math.PI/2),0.02,0.05,0.05),new PlannerBackend.Footprint(0.4,0.4),
            new PlannerBackend.Constraints(3,3,4),new PlannerBackend.Bounds(0,0,6,4),List.of(),1_000,1_000_000,
            new PlannerBackend.SearchBudget(System.nanoTime(),1_000_000_000L),cancellation);
    }
    private static PlannerBackend.Request modify(PlannerBackend.Request r,Pose2 start,Velocity2 velocity,PlannerBackend.SearchBudget budget,List<PlannerBackend.Obstacle> obstacles) {
        return new PlannerBackend.Request(r.requestId(),r.taskId(),r.epoch(),r.snapshotId(),r.obstacleMapVersion(),r.field(),start,velocity,r.goal(),r.footprint(),r.constraints(),r.bounds(),obstacles,r.issuedUs(),r.validUntilUs(),budget,r.cancellation());
    }
    private static void adapter() {
        AtomicLong robot=new AtomicLong(2_000);AStarPlannerBackend backend=new AStarPlannerBackend(robot::get);
        PlannerBackend.Request r=request("adapter",3,4,5,()->false);PlannerBackend.Result result=backend.plan(r);
        require(result.status()==PlannerBackend.Status.SUCCESS,result.detail());require(result.trajectory()==null,"Search duration confused with timed motion");
        require(result.requestId().equals(r.requestId())&&result.taskId().equals(r.taskId())&&result.snapshotId()==3&&result.obstacleMapVersion()==4&&result.epoch()==5,"Provenance lost");
        require(result.validUntilUs()==r.validUntilUs()&&result.solverDurationNanos()>=0,"Validity/duration lost");
        r=modify(r,r.startPose(),new Velocity2(new Vec2(-1,1),2),r.budget(),List.of(new PlannerBackend.Obstacle("static",new Vec2(3,2),0.6,0.1,r.validUntilUs(),false)));
        result=backend.plan(r);require(result.status()==PlannerBackend.Status.SUCCESS,result.detail());require(result.trajectory()==null,"Invented velocity profile");
        require(Arrays.stream(PlannerBackend.Result.class.getRecordComponents()).noneMatch(c->c.getName().toLowerCase().contains("complete")),"Physical success authority introduced");
        robot.set(999);require(backend.plan(r).status()==PlannerBackend.Status.STALE_RESULT,"Robot clock rollback admitted");
        robot.set(2_000);PlannerBackend.Request still=new PlannerBackend.Request("still",r.taskId(),r.epoch(),r.snapshotId(),r.obstacleMapVersion(),r.field(),r.startPose(),Velocity2.zero(),
            new PlannerBackend.Goal(r.startPose(),0,0,0),r.footprint(),r.constraints(),r.bounds(),List.of(),r.issuedUs(),r.validUntilUs(),
            new PlannerBackend.SearchBudget(System.nanoTime(),1_000_000_000L),()->false);
        result=backend.plan(still);require(result.status()==PlannerBackend.Status.SUCCESS&&result.path().points().size()==2&&result.path().points().get(0).equals(result.path().points().get(1)),"Stationary v1 contract path failed");
    }
    private static void adapterRejects() {
        AStarPlannerBackend backend=new AStarPlannerBackend(()->2_000);PlannerBackend.Request r=request("reject",1,1,1,()->false);
        require(backend.plan(modify(r,new Pose2(new Vec2(-1,2),0),r.startVelocity(),r.budget(),List.of())).status()==PlannerBackend.Status.NO_PATH,"Out of bounds adapter start accepted");
        require(backend.plan(modify(r,r.startPose(),new Velocity2(new Vec2(4,0),0),r.budget(),List.of())).status()==PlannerBackend.Status.INVALID_INPUT,"Invalid starting speed");
        require(backend.plan(modify(r,r.startPose(),new Velocity2(new Vec2(0,0),5),r.budget(),List.of())).status()==PlannerBackend.Status.INVALID_INPUT,"Invalid starting angular speed");
        require(backend.plan(modify(r,r.startPose(),r.startVelocity(),new PlannerBackend.SearchBudget(System.nanoTime()-2_000_000_000L,1),List.of())).status()==PlannerBackend.Status.TIMEOUT,"Old absolute budget restarted");
        require(backend.plan(request("cancel",1,1,1,()->true)).status()==PlannerBackend.Status.CANCELLED,"API cancellation ignored");
        PlannerBackend.Obstacle shortEnvelope=new PlannerBackend.Obstacle("expires",new Vec2(3,2),0.2,0.1,3_000,true);
        throwsIllegal(()->modify(r,r.startPose(),r.startVelocity(),r.budget(),List.of(shortEnvelope)));
    }
    private static PlannerBackend.Result fakeSuccess(PlannerBackend.Request r) {
        return new PlannerBackend.Result(r.requestId(),r.taskId(),r.epoch(),r.snapshotId(),r.obstacleMapVersion(),PlannerBackend.Status.SUCCESS,
            new PlannerBackend.GeometricPath(List.of(r.startPose(),r.goal().pose())),null,1,r.validUntilUs(),"test-backend","geometry only");
    }
    private static void latestWins() throws Exception {
        CountDownLatch entered=new CountDownLatch(1),release=new CountDownLatch(1);AtomicInteger calls=new AtomicInteger();
        PlannerBackend ignoresCancellation=r->{ if(calls.incrementAndGet()==1){entered.countDown();try{require(release.await(2,TimeUnit.SECONDS),"Release timeout");}catch(InterruptedException ex){Thread.currentThread().interrupt();}} return fakeSuccess(r); };
        try(LatestRequestPlanner gate=new LatestRequestPlanner(ignoresCancellation)) {
            gate.setEnabled(true);CompletableFuture<PlannerBackend.Result> old=gate.submit(request("old",1,1,1,()->false));require(entered.await(2,TimeUnit.SECONDS),"Worker not entered");
            CompletableFuture<PlannerBackend.Result> superseded=gate.submit(request("middle",1,1,1,()->false));
            CompletableFuture<PlannerBackend.Result> newest=gate.submit(request("new",1,1,1,()->false));
            require(gate.currentProposal(2_000).isEmpty(),"Old path retained after replacement");release.countDown();
            require(newest.get(2,TimeUnit.SECONDS).status()==PlannerBackend.Status.SUCCESS,"New request failed");
            require(old.get(2,TimeUnit.SECONDS).status()!=PlannerBackend.Status.SUCCESS,"Older ignored cancellation and published");
            require(superseded.get(2,TimeUnit.SECONDS).status()!=PlannerBackend.Status.SUCCESS,"Pending queue not latest-only");
            require(gate.currentProposal(2_000).orElseThrow().requestId().equals("new"),"Newer proposal replaced by older");
            require(calls.get()==2,"More than one pending request queued");
        } finally { release.countDown(); }
    }
    private static void invalidation() throws Exception {
        try(LatestRequestPlanner gate=new LatestRequestPlanner(ContractSuite::fakeSuccess)) {
            require(gate.submit(request("disabled",1,1,1,()->false)).get().status()!=PlannerBackend.Status.SUCCESS,"Initially enabled");
            gate.setEnabled(true);gate.submit(request("first",1,1,1,()->false)).get(2,TimeUnit.SECONDS);require(gate.currentProposal(2_000).isPresent(),"No first proposal");
            gate.setEnabled(false);require(gate.currentProposal(2_000).isEmpty(),"Disable retains path");
            gate.setEnabled(true);gate.submit(request("second",1,1,1,()->false)).get(2,TimeUnit.SECONDS);gate.cancel();require(gate.currentProposal(2_000).isEmpty(),"Cancel retains path");
            for(String reason:List.of("pose reset","source restart","task replaced","map epoch changed")) {
                gate.submit(request(reason,1,1,1,()->false)).get(2,TimeUnit.SECONDS);gate.invalidate(reason);require(gate.currentProposal(2_000).isEmpty(),"Invalidation retains path: "+reason);
            }
            gate.invalidate(new LatestRequestPlanner.Context(2,2,2,FIELD));
            require(gate.submit(request("old-map",1,1,1,()->false)).get().status()==PlannerBackend.Status.STALE_RESULT,"Obsolete snapshot accepted");
            gate.submit(request("new-map",2,2,2,()->false)).get(2,TimeUnit.SECONDS);require(gate.currentProposal(2_000).isPresent(),"Current map rejected");
            require(gate.currentProposal(1_000_001).isEmpty(),"Expired path retained");
        }
    }
}
