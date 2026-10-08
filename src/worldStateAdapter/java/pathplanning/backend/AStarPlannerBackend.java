package pathplanning.backend;

import java.util.*;
import java.util.function.LongSupplier;
import org.frcworldstate.core.Geometry.*;
import org.frcworldstate.core.PlannerBackend;
import pathplanning.geometric.Geometry;
import pathplanning.geometric.GeometricSolver;

/** Implements the pinned frc-planner/1 SPI. World-State has no dependency on this library.
 * No WPILib, NetworkTables, PathPlanner, drivetrain or objective-selection dependency. */
public final class AStarPlannerBackend implements PlannerBackend {
    public static final String BACKEND_ID="1086-geometric-a-star/1";
    public record Options(double gridResolutionM, double clearanceM, int maxCells,
                          int maxExpanded, int maxQueueEntries, boolean simplify) {
        public Options {
            if(!Double.isFinite(gridResolutionM)||gridResolutionM<0.001||gridResolutionM>1 ||
               !Double.isFinite(clearanceM)||clearanceM<0 || maxCells<=0||maxCells>1_000_000 ||
               maxExpanded<=0||maxExpanded>1_000_000 || maxQueueEntries<=0||maxQueueEntries>1_000_000)
                throw new IllegalArgumentException("Invalid bounded solver options");
        }
        public static Options conservativeDefault() { return new Options(0.1,0.02,250_000,100_000,250_000,true); }
    }
    private final Options options;
    private final LongSupplier nanoTime, robotTimeUs;
    private final GeometricSolver solver;
    /** Caller supplies the robot timestamp domain explicitly; never equate it to System.nanoTime. */
    public AStarPlannerBackend(LongSupplier robotTimeUs) { this(Options.conservativeDefault(),System::nanoTime,robotTimeUs); }
    public AStarPlannerBackend(Options options,LongSupplier nanoTime,LongSupplier robotTimeUs) {
        this.options=Objects.requireNonNull(options);this.nanoTime=Objects.requireNonNull(nanoTime);
        this.robotTimeUs=Objects.requireNonNull(robotTimeUs);this.solver=new GeometricSolver(nanoTime);
    }
    @Override public Result plan(Request request) {
        Objects.requireNonNull(request,"request"); long began=nanoTime.getAsLong();
        if(request.cancellation().cancelled()||Thread.currentThread().isInterrupted()) return failure(request,Status.CANCELLED,began,"Cancelled request");
        long nowUs=robotTimeUs.getAsLong();
        if(nowUs<request.issuedUs() || nowUs>=request.validUntilUs()) return failure(request,Status.STALE_RESULT,began,"Request outside robot-clock validity interval");
        long elapsed=began-request.budget().startedNanos();
        if(elapsed<0 || request.budget().limitNanos()>5_000_000_000L) return failure(request,Status.INVALID_INPUT,began,"Invalid monotonic budget origin/limit");
        long remaining=request.budget().limitNanos()-elapsed;
        if(remaining<=0) return failure(request,Status.TIMEOUT,began,"Budget already expired before adapter work");
        try {
            Geometry.Input input=convert(request,remaining);
            Geometry.Output output=solver.plan(input,()->request.cancellation().cancelled() || request.budget().expired(nanoTime.getAsLong()));
            if(output.status()!=Geometry.Status.SUCCESS) {
                Status status=mapStatus(output.status());
                if(status==Status.CANCELLED && !request.cancellation().cancelled() && request.budget().expired(nanoTime.getAsLong())) status=Status.TIMEOUT;
                return failure(request,status,began,output.status()+": "+output.detail());
            }
            List<Pose2> poses=new ArrayList<>();
            for(Geometry.Pose pose:output.poses()) {
                poll(request);
                poses.add(new Pose2(new Vec2(pose.x()+request.bounds().minXM(),pose.y()+request.bounds().minYM()),pose.headingRadians()));
            }
            // v1 requires at least two points. Duplicate stationary endpoint means geometry, never a motion duration.
            if(poses.size()==1) poses.add(poses.get(0));
            if(poses.size()>10000) return failure(request,Status.NO_PATH,began,"Result exceeds public contract path capacity");
            List<Geometry.Pose> converted=new ArrayList<>();
            for(Pose2 pose:poses) converted.add(toLocal(pose,request.bounds()));
            if(!GeometricSolver.validatePath(input.map(),converted,input.footprint().enclosingRadius(),()->poll(request)))
                return failure(request,Status.NO_PATH,began,"Collision validation failed after API conversion");
            poll(request); nowUs=robotTimeUs.getAsLong();
            if(nowUs<request.issuedUs() || nowUs>=request.validUntilUs()) return failure(request,Status.STALE_RESULT,began,"Expired before publication");
            return new Result(request.requestId(),request.taskId(),request.epoch(),request.snapshotId(),request.obstacleMapVersion(),
                Status.SUCCESS,new GeometricPath(poses),null,duration(began),request.validUntilUs(),BACKEND_ID,
                "Geometry only; field="+request.field()+"; snapshot immutable; robot footprint inflated once; no physical completion");
        } catch(Cancelled e) { return failure(request,Status.CANCELLED,began,"Cancelled during conversion/validation"); }
          catch(Expired e) { return failure(request,Status.TIMEOUT,began,"Monotonic adapter budget exhausted"); }
          catch(IllegalArgumentException e) { return failure(request,Status.INVALID_INPUT,began,e.getMessage()); }
    }
    private Geometry.Input convert(Request request,long remaining) {
        Bounds b=request.bounds(); double width=b.maxXM()-b.minXM(),height=b.maxYM()-b.minYM();
        if(!Double.isFinite(width)||!Double.isFinite(height)||width>1000||height>1000) throw new IllegalArgumentException("Excessive field bounds");
        long columns=(long)Math.ceil(width/options.gridResolutionM()),rows=(long)Math.ceil(height/options.gridResolutionM());
        if(columns<=0 || rows<=0 || columns*rows>options.maxCells()) throw new IllegalArgumentException("Map exceeds bounded cell capacity");
        double startSpeed=request.startVelocity().linearMps().norm();
        if(!Double.isFinite(startSpeed)||startSpeed>request.constraints().maxSpeedMps() ||
            Math.abs(request.startVelocity().angularRadPerSec())>request.constraints().maxAngularSpeedRadPerSec())
            throw new IllegalArgumentException("Start velocity exceeds verified constraints");
        List<Geometry.Envelope> envelopes=new ArrayList<>(); Set<String> ids=new HashSet<>();
        for(Obstacle obstacle:request.obstacles()) {
            poll(request);
            if(!ids.add(obstacle.id())) throw new IllegalArgumentException("Duplicate obstacle identity");
            double radius=obstacle.envelopeRadiusM();
            if(!Double.isFinite(radius) || radius<=0) throw new IllegalArgumentException("Empty/nonfinite object envelope");
            double x=obstacle.centerM().x()-b.minXM(),y=obstacle.centerM().y()-b.minYM();
            // API circle -> circumscribed AABB is conservative. Geometry, uncertainty, age and swept forecast
            // belong to World-State's raw envelope. The solver alone adds robot footprint + configured clearance.
            envelopes.add(new Geometry.Envelope(obstacle.id(),new Geometry.Rectangle(x-radius,y-radius,x+radius,y+radius),-1));
        }
        poll(request);
        Geometry.MapSnapshot map=new Geometry.MapSnapshot(width,height,options.gridResolutionM(),(int)columns,(int)rows,
            new boolean[(int)(columns*rows)],envelopes);
        Constraints c=request.constraints(); Goal g=request.goal();
        return new Geometry.Input(toLocal(request.startPose(),b),toLocal(g.pose(),b),
            new Geometry.Footprint(request.footprint().lengthM(),request.footprint().widthM(),options.clearanceM()),
            new Geometry.Constraints(c.maxSpeedMps(),c.maxAccelerationMps2(),c.maxAngularSpeedRadPerSec(),startSpeed,0,g.positionToleranceM(),g.headingToleranceRad()),
            map,new Geometry.Budget(remaining,options.maxExpanded(),options.maxQueueEntries(),options.maxCells()),options.simplify());
    }
    private static Geometry.Pose toLocal(Pose2 p,Bounds b) { return new Geometry.Pose(p.position().x()-b.minXM(),p.position().y()-b.minYM(),p.headingRad()); }
    private void poll(Request r) {
        if(r.cancellation().cancelled()||Thread.currentThread().isInterrupted()) throw new Cancelled();
        if(r.budget().expired(nanoTime.getAsLong())) throw new Expired();
    }
    private long duration(long began) { return Math.max(0,nanoTime.getAsLong()-began); }
    private Result failure(Request r,Status status,long began,String detail) {
        return new Result(r.requestId(),r.taskId(),r.epoch(),r.snapshotId(),r.obstacleMapVersion(),status,null,null,duration(began),r.validUntilUs(),BACKEND_ID,detail);
    }
    private static Status mapStatus(Geometry.Status status) {
        return switch(status) {
            case CANCELLED -> Status.CANCELLED;
            case TIMEOUT, RESOURCE_LIMIT -> Status.TIMEOUT;
            case INVALID_INPUT -> Status.INVALID_INPUT;
            default -> Status.NO_PATH;
        };
    }
    private static final class Cancelled extends RuntimeException { private static final long serialVersionUID = 1L; }
    private static final class Expired extends RuntimeException { private static final long serialVersionUID = 1L; }
}
