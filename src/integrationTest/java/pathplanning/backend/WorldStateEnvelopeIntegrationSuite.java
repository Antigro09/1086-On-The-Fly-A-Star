package pathplanning.backend;

import java.util.List;
import java.util.Map;
import org.frcworldstate.core.ObstacleEnvelopeBuilder;
import org.frcworldstate.core.PlannerBackend;
import org.frcworldstate.core.PlannerValidation;
import org.frcworldstate.core.World;
import static org.frcworldstate.core.Geometry.*;

/** Direct CPU-only integration with exact owner-exported envelope builder and validator. */
public final class WorldStateEnvelopeIntegrationSuite {
    private static int checks;
    private static final long ISSUED=1_000_000, EXPIRY=1_200_000;
    private static final FieldIdentity FIELD=new FieldIdentity("offseason","cross-repo-synthetic","r1");
    private static void check(boolean condition,String detail) { if(!condition)throw new AssertionError(detail);checks++; }
    private static ObstacleEnvelopeBuilder builder() {
        return new ObstacleEnvelopeBuilder(new ObstacleEnvelopeBuilder.Config(
            Map.of("robot",new ObstacleEnvelopeBuilder.ClassGeometry(0.3,1,true),
                   "fixed",new ObstacleEnvelopeBuilder.ClassGeometry(0.3,0,false)),
            2,300_000,100_000,300_000,20_000,0.03));
    }
    private static World.WorldSnapshot snapshot(String objectClass,long measurementUs,boolean moving) {
        World.EgoState ego=new World.EgoState(ISSUED,new Pose2(new Vec2(0.6,2),0),Velocity2.zero(),new Uncertainty(0,0,0),true);
        double ageSec=(ISSUED-measurementUs)/1e6;
        World.ObjectTrack track=new World.ObjectTrack(9,3,objectClass,new Vec2(2.5+(moving?ageSec:0),2),
            new Vec2(moving?1:0,0),ISSUED,measurementUs,0.9,new Uncertainty(0.01,0,0.01),
            World.TrackLifecycle.COASTING,World.EstimateKind.PROPAGATED,List.of());
        return new World.WorldSnapshot(7,3,8,FIELD,ego,List.of(track));
    }
    private static PlannerBackend.Request request(World.WorldSnapshot snapshot,List<PlannerBackend.Obstacle> obstacles) {
        return new PlannerBackend.Request("owner-envelope-integration","task",snapshot.epoch(),snapshot.id(),snapshot.obstacleMapVersion(),snapshot.field(),
            snapshot.ego().fieldPose(),snapshot.ego().fieldVelocity(),new PlannerBackend.Goal(new Pose2(new Vec2(5.4,2),Math.PI/2),0.02,0.05,0.05),
            new PlannerBackend.Footprint(0.4,0.4),new PlannerBackend.Constraints(3,3,4),new PlannerBackend.Bounds(0,0,6,4),
            obstacles,ISSUED,EXPIRY,new PlannerBackend.SearchBudget(System.nanoTime(),1_000_000_000),()->false);
    }
    public static void main(String[] args) {
        World.WorldSnapshot world=snapshot("robot",800_000,true);
        ObstacleEnvelopeBuilder.Decision envelopes=builder().build(world,ISSUED,EXPIRY,true);
        check(envelopes.usable()&&envelopes.obstacles().size()==1,"Owner envelope was not usable");
        PlannerBackend.Obstacle obstacle=envelopes.obstacles().get(0);
        check(Math.abs(obstacle.centerM().x()-2.5)<1e-12,"CV measured-center reconstruction changed");
        check(Math.abs(obstacle.radiusM()-0.3)<1e-12,"Physical dimension radius lost");
        // 2sigma*sqrt(.01) + 1m/s * (.4s capture-through-expiry + .02 timing + .03 latency)
        check(Math.abs(obstacle.uncertaintyMarginM()-0.65)<1e-12,"Age/uncertainty/motion/timing/latency expansion changed");
        check(Math.abs(obstacle.envelopeRadiusM()-0.95)<1e-12,"Robot footprint was wrongly pre-inflated upstream");
        check(obstacle.validUntilUs()==EXPIRY&&obstacle.dynamic(),"Envelope horizon/dynamic marker lost");
        PlannerBackend.Request request=request(world,envelopes.obstacles());
        PlannerBackend.Result result=new AStarPlannerBackend(()->ISSUED+10_000).plan(request);
        check(result.status()==PlannerBackend.Status.SUCCESS,"A* rejected owner-computed envelope: "+result.detail());
        check(result.trajectory()==null,"Geometric integration fabricated motion timing");
        check(result.requestId().equals(request.requestId())&&result.snapshotId()==world.id()&&result.obstacleMapVersion()==world.obstacleMapVersion(),"Cross-repo result provenance mismatch");
        check(result.path().points().stream().anyMatch(p->Math.abs(p.position().y()-2)>1),"Path ignored complete swept envelope");
        PlannerBackend.Result checked=PlannerValidation.validate(request,result,world,world.epoch(),world.obstacleMapVersion(),ISSUED+10_000);
        check(checked.status()==PlannerBackend.Status.SUCCESS,"Owner validator rejected A* route: "+checked.detail());
        check(PlannerValidation.validate(request,result,world,world.epoch(),world.obstacleMapVersion()+1,ISSUED+10_000).status()==PlannerBackend.Status.STALE_RESULT,"Changed owner map accepted");
        check(PlannerValidation.validate(request,result,world,world.epoch()+1,world.obstacleMapVersion(),ISSUED+10_000).status()==PlannerBackend.Status.STALE_RESULT,"Changed owner epoch accepted");
        World.WorldSnapshot wrongField=new World.WorldSnapshot(world.id(),world.epoch(),world.obstacleMapVersion(),new FieldIdentity("other","other","other"),world.ego(),world.tracks());
        check(PlannerValidation.validate(request,result,wrongField,world.epoch(),world.obstacleMapVersion(),ISSUED+10_000).status()==PlannerBackend.Status.STALE_RESULT,"Changed owner field accepted");
        PlannerBackend.TimedTrajectory uncheckedTiming=new PlannerBackend.TimedTrajectory(List.of(
            new PlannerBackend.TimedPoint(0,request.startPose(),Velocity2.zero()),new PlannerBackend.TimedPoint(1,request.goal().pose(),Velocity2.zero())));
        PlannerBackend.Result timed=new PlannerBackend.Result(result.requestId(),result.taskId(),result.epoch(),result.snapshotId(),result.obstacleMapVersion(),
            result.status(),result.path(),uncheckedTiming,result.solverDurationNanos(),result.validUntilUs(),result.backendId(),"unqualified timing fixture");
        check(PlannerValidation.validate(request,timed,world,world.epoch(),world.obstacleMapVersion(),ISSUED+10_000).status()==PlannerBackend.Status.INVALID_INPUT,"Unqualified timed data passed owner geometry-only gate");
        ObstacleEnvelopeBuilder.Decision aged=builder().build(snapshot("robot",600_000,true),ISSUED,EXPIRY,true);
        check(!aged.usable()&&aged.obstacles().isEmpty()&&aged.status()==ObstacleEnvelopeBuilder.Status.STALE_INFORMATION,"Stale last measurement became usable occupancy");
        ObstacleEnvelopeBuilder.Decision unknown=builder().build(snapshot("unconfigured",800_000,false),ISSUED,EXPIRY,true);
        check(!unknown.usable()&&unknown.obstacles().isEmpty()&&unknown.status()==ObstacleEnvelopeBuilder.Status.UNKNOWN_GEOMETRY,"Missing dimensions became safe stationary object");
        World.WorldSnapshot empty=new World.WorldSnapshot(world.id(),world.epoch(),world.obstacleMapVersion(),world.field(),world.ego(),List.of());
        check(!builder().build(empty,ISSUED,EXPIRY,false).usable(),"Empty observations inferred free-space coverage");
        check(builder().build(empty,ISSUED,EXPIRY,true).usable(),"Independent fresh coverage rejected");
        ObstacleEnvelopeBuilder.Decision fixed=builder().build(snapshot("fixed",800_000,false),ISSUED,EXPIRY,true);
        check(fixed.usable()&&Math.abs(fixed.obstacles().get(0).envelopeRadiusM()-0.5)<1e-12&&!fixed.obstacles().get(0).dynamic(),"Static geometry/uncertainty envelope changed");
        System.out.println("PASS: "+checks+" World-State envelope/adapter/owner-validator checks; CPU synthetic only");
    }
}
