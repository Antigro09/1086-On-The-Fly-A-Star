package pathplanning.backend;

import java.util.ArrayList;
import java.util.List;
import java.util.HashSet;
import java.util.Set;
import java.util.Objects;
import java.util.function.LongSupplier;
import org.frcworldstate.core.Geometry.Pose2;
import org.frcworldstate.core.PlannerBackend;
import pathplanning.geometric.Geometry;
import pathplanning.motion.BezierSmoother;
import pathplanning.motion.MotionPath;
import pathplanning.motion.TimeParameterizer;

/** Bounded route/curve/time optimization. The owner SPI remains geometry-only; timed motion is a separate proposal.
 * No drivetrain follower, motor model, transport or physical objective authority lives here. */
public final class CurvedMotionPlanner {
    public static final String BACKEND_ID = "1086-curved-motion/1";
    public record MotionLimits(double maxCentripetalAcceleration, double maxAngularAcceleration, double endSpeed) {
        public MotionLimits {
            if (!Double.isFinite(maxCentripetalAcceleration) || maxCentripetalAcceleration<=0 || maxCentripetalAcceleration>30
                    || !Double.isFinite(maxAngularAcceleration) || maxAngularAcceleration<=0 || maxAngularAcceleration>60
                    || !Double.isFinite(endSpeed) || endSpeed<0 || endSpeed>20)
                throw new IllegalArgumentException("Invalid bounded motion limits");
        }
    }
    public record Proposal(PlannerBackend.Result geometry, MotionPath.Curve curve,
                           MotionPath.Profile profile, int candidateCount, long plannedStartUs,
                           double validatedPrefixSec, long planningDurationNanos) {}
    private final LongSupplier nanoTime, robotTimeUs;
    private final List<AStarPlannerBackend> routes;
    private static final double RESOLUTION=.1, BASE_CLEARANCE=.02;
    private static final int MAX_PROFILE_POINTS=768;

    public CurvedMotionPlanner(LongSupplier nanoTime, LongSupplier robotTimeUs) {
        this.nanoTime=Objects.requireNonNull(nanoTime); this.robotTimeUs=Objects.requireNonNull(robotTimeUs);
        List<AStarPlannerBackend> planners=new ArrayList<>();
        // More clearance can permit a larger, faster bend, even when it produces a longer route.
        for (double clearance : new double[]{BASE_CLEARANCE,.10,.25})
            planners.add(new AStarPlannerBackend(new AStarPlannerBackend.Options(RESOLUTION,clearance,250_000,100_000,250_000,true),nanoTime,robotTimeUs));
        routes=List.copyOf(planners);
    }

    public Proposal plan(PlannerBackend.Request request, MotionLimits extra) {
        return plan(request,extra,true);
    }
    /** Rolling replanning: only the prefix before geometry.validUntilUs is forecast-qualified.
     * The remaining profile is a proposal for future replans, never authority to execute beyond expiry. */
    public Proposal planRecedingHorizon(PlannerBackend.Request request, MotionLimits extra) {
        return plan(request,extra,false);
    }
    private Proposal plan(PlannerBackend.Request request, MotionLimits extra, boolean requireFullHorizon) {
        Objects.requireNonNull(request); Objects.requireNonNull(extra);
        long began=nanoTime.getAsLong(); Selection best=new Selection();
        Geometry.MapSnapshot map=null;
        PlannerBackend.Result firstFailure=null; boolean hadSeed=false,cutoff=false;
        if (began-request.budget().startedNanos()<0 || request.budget().limitNanos()>5_000_000_000L)
            return failure(request,PlannerBackend.Status.INVALID_INPUT,began,"Invalid monotonic budget origin/limit");
        // Reserve publication/conversion time. Cancellation and the original deadline always override a proposal.
        long refinementLimit=Math.max(1,request.budget().limitNanos()*4/5);
        Runnable poll=()->{
            if (request.cancellation().cancelled() || Thread.currentThread().isInterrupted()) throw new Cancelled();
            long elapsed=nanoTime.getAsLong()-request.budget().startedNanos();
            if (elapsed<0 || elapsed>=request.budget().limitNanos()) throw new Expired();
            if (elapsed>=refinementLimit) throw new RefineDone();
        };
        try {
            if (extra.endSpeed()>request.constraints().maxSpeedMps()) throw new IllegalArgumentException("Goal speed exceeds maximum speed");
            if (extra.endSpeed()>request.goal().velocityToleranceMps())
                throw new IllegalArgumentException("Goal end speed exceeds the request's goal velocity tolerance");
            if (Math.max(Math.max(Math.abs(request.bounds().minXM()),Math.abs(request.bounds().maxXM())),
                    Math.max(Math.abs(request.bounds().minYM()),Math.abs(request.bounds().maxYM())))>10000)
                throw new IllegalArgumentException("Motion field origin/extent exceeds bounded metric coordinates");
            double startSpeed=request.startVelocity().linearMps().norm();
            Double tangent=startSpeed>1e-9 ? Math.atan2(request.startVelocity().linearMps().y(),request.startVelocity().linearMps().x()) : null;
            MotionPath.Limits limits=new MotionPath.Limits(request.constraints().maxSpeedMps(),request.constraints().maxAccelerationMps2(),
                    extra.maxCentripetalAcceleration(),request.constraints().maxAngularSpeedRadPerSec(),extra.maxAngularAcceleration(),
                    startSpeed,extra.endSpeed(),request.startPose().headingRad(),request.goal().pose().headingRad(),
                    request.startVelocity().angularRadPerSec(),0);
            for (AStarPlannerBackend backend:routes) {
                poll.run();
                long remaining=refinementLimit-(nanoTime.getAsLong()-request.budget().startedNanos());
                if (remaining<=0) break;
                // All route searches share the original elapsed budget; no new time allowance per candidate.
                PlannerBackend.Request seedRequest=new PlannerBackend.Request(request.requestId(),request.taskId(),request.epoch(),request.snapshotId(),
                        request.obstacleMapVersion(),request.field(),request.startPose(),request.startVelocity(),request.goal(),request.footprint(),
                        request.constraints(),request.bounds(),request.obstacles(),request.issuedUs(),request.validUntilUs(),
                        new PlannerBackend.SearchBudget(nanoTime.getAsLong(),remaining),request.cancellation());
                PlannerBackend.Result seed=backend.plan(seedRequest);
                if (seed.status()!=PlannerBackend.Status.SUCCESS) {
                    if (firstFailure==null) firstFailure=seed;
                    if (seed.status()==PlannerBackend.Status.CANCELLED) throw new Cancelled();
                    if (seed.status()==PlannerBackend.Status.INVALID_INPUT) throw new IllegalArgumentException(seed.detail());
                    continue;
                }
                hadSeed=true;
                if (seed.path().points().size()>512) { best.rejection="Route exceeds bounded display capacity"; continue; }
                if (map==null) map=map(request,poll);
                List<Geometry.Pose> local=new ArrayList<>();
                for (Pose2 p:seed.path().points()) local.add(new Geometry.Pose(p.position().x()-request.bounds().minXM(),
                        p.position().y()-request.bounds().minYM(),p.headingRad()));
                BezierSmoother.visitCandidates(local,map,request.footprint().boundingRadiusM()+BASE_CLEARANCE,tangent,poll,curve -> {
                    poll.run();
                    if (!best.seen.add(curve.segments())) return;
                    try {
                        MotionPath.Curve canonical=TimeParameterizer.integratedCurve(curve,poll);
                        MotionPath.Profile profile=TimeParameterizer.profile(canonical,limits,poll);
                        if (profile.samples().size()>MAX_PROFILE_POINTS) { best.rejection="Motion profile exceeds bounded transport sample capacity"; return; }
                        if (requireFullHorizon && Math.ceil(profile.durationSec()*1_000_000)>=request.validUntilUs()-robotTimeUs.getAsLong()) {
                            best.rejection="Motion duration exceeds the remaining obstacle forecast/request validity horizon"; return;
                        }
                        best.count++;
                        if (best.profile==null || profile.durationSec()<best.profile.durationSec()-1e-9) {
                            best.profile=profile; best.curve=canonical; best.seed=seed;
                        }
                    } catch (IllegalArgumentException infeasible) { best.rejection=infeasible.getMessage(); }
                });
            }
        } catch (RefineDone complete) {
            // Retain an already validated incumbent; never retain work after cancellation or actual expiry.
            cutoff=true;
        } catch (Cancelled cancelled) { return failure(request,PlannerBackend.Status.CANCELLED,began,"Motion planning cancelled");
        } catch (Expired expired) { return failure(request,PlannerBackend.Status.TIMEOUT,began,"Shared route/curve/timing budget expired");
        } catch (IllegalArgumentException invalid) { return failure(request,PlannerBackend.Status.INVALID_INPUT,began,invalid.getMessage()); }
        if (request.cancellation().cancelled() || Thread.currentThread().isInterrupted())
            return failure(request,PlannerBackend.Status.CANCELLED,began,"Cancelled before motion publication");
        if (request.budget().expired(nanoTime.getAsLong())) return failure(request,PlannerBackend.Status.TIMEOUT,began,"Expired before motion publication");
        long now=robotTimeUs.getAsLong();
        if (now<request.issuedUs() || now>=request.validUntilUs()) return failure(request,PlannerBackend.Status.STALE_RESULT,began,"Robot-clock validity expired");
        if (best.profile==null) return failure(request,cutoff ? PlannerBackend.Status.TIMEOUT : !hadSeed && firstFailure!=null ? firstFailure.status() : PlannerBackend.Status.NO_PATH,began,
                !hadSeed && firstFailure!=null ? firstFailure.detail() : best.rejection);
        double dx=request.bounds().minXM(),dy=request.bounds().minYM();
        Runnable finalPoll=()->{
            if (request.cancellation().cancelled() || Thread.currentThread().isInterrupted()) throw new Cancelled();
            if (request.budget().expired(nanoTime.getAsLong())) throw new Expired();
        };
        MotionPath.Curve global; List<MotionPath.Sample> samples=new ArrayList<>();
        try {
        global=translate(best.curve,dx,dy,finalPoll);
        // Recheck the actual converted controls, not the seed polyline or its cached knots.
        // This extra margin bounds coordinate conversion/evaluation roundoff, not another robot inflation.
        double coordinateScale=Math.max(1,Math.max(Math.abs(dx)+map.width(),Math.abs(dy)+map.height()));
        double numericGuard=8192*Math.ulp(coordinateScale);
        MotionPath.Curve convertedLocal=translate(global,-dx,-dy,finalPoll);
        if (!BezierSmoother.revalidate(convertedLocal,map,request.footprint().boundingRadiusM()+BASE_CLEARANCE+numericGuard,finalPoll))
            return failure(request,PlannerBackend.Status.NO_PATH,began,"Cubic footprint validation failed after coordinate conversion");
        for (MotionPath.Sample p:best.profile.samples()) { finalPoll.run(); samples.add(new MotionPath.Sample(p.timeSec(),p.distanceM(),p.segment(),p.u(),
                p.x()+dx,p.y()+dy,p.headingRadians(),p.tangentRadians(),p.speedMps(),p.accelerationMps2(),p.vxMps(),p.vyMps(),p.omegaRadPerSec())); }
        finalPoll.run();
        } catch (Cancelled cancelled) { return failure(request,PlannerBackend.Status.CANCELLED,began,"Cancelled during motion conversion");
        } catch (Expired expired) { return failure(request,PlannerBackend.Status.TIMEOUT,began,"Deadline during motion conversion"); }
        long plannedStartUs=robotTimeUs.getAsLong();
        if (plannedStartUs<request.issuedUs() || plannedStartUs>=request.validUntilUs()
                || requireFullHorizon && Math.ceil(best.profile.durationSec()*1_000_000)>=request.validUntilUs()-plannedStartUs)
            return failure(request,PlannerBackend.Status.STALE_RESULT,began,"Insufficient remaining forecast horizon for motion at publication");
        MotionPath.Profile profile=new MotionPath.Profile(samples,best.profile.durationSec(),best.profile.peakSpeedMps(),best.profile.detail());
        long duration=Math.max(0,nanoTime.getAsLong()-began);
        PlannerBackend.Result result=new PlannerBackend.Result(request.requestId(),request.taskId(),request.epoch(),request.snapshotId(),request.obstacleMapVersion(),
                PlannerBackend.Status.SUCCESS,best.seed.path(),null,duration,request.validUntilUs(),BACKEND_ID,
                "Fastest of "+best.count+" evaluated feasible curve profiles; bounded search, not a global optimality proof; geometry SPI only");
        if (request.cancellation().cancelled() || Thread.currentThread().isInterrupted()) return failure(request,PlannerBackend.Status.CANCELLED,began,"Cancelled before return");
        if (request.budget().expired(nanoTime.getAsLong())) return failure(request,PlannerBackend.Status.TIMEOUT,began,"Deadline before return");
        long returnUs=robotTimeUs.getAsLong();
        if (returnUs<plannedStartUs || returnUs>=request.validUntilUs()
                || requireFullHorizon && Math.ceil(profile.durationSec()*1_000_000)>=request.validUntilUs()-returnUs)
            return failure(request,PlannerBackend.Status.STALE_RESULT,began,"Forecast horizon changed before return");
        double prefix=Math.min(profile.durationSec(),Math.max(0,request.validUntilUs()-returnUs-1)/1_000_000.0);
        return new Proposal(result,global,profile,best.count,plannedStartUs,prefix,duration);
    }

    private Geometry.MapSnapshot map(PlannerBackend.Request r,Runnable poll) {
        double width=r.bounds().maxXM()-r.bounds().minXM(),height=r.bounds().maxYM()-r.bounds().minYM();
        int cols=(int)Math.ceil(width/RESOLUTION),rows=(int)Math.ceil(height/RESOLUTION);
        if (cols<=0 || rows<=0 || (long)cols*rows>250_000) throw new IllegalArgumentException("Motion map exceeds bounded capacity");
        List<Geometry.Envelope> envelopes=new ArrayList<>();
        for (PlannerBackend.Obstacle o:r.obstacles()) {
            poll.run(); double x=o.centerM().x()-r.bounds().minXM(),y=o.centerM().y()-r.bounds().minYM(),radius=o.envelopeRadiusM();
            envelopes.add(new Geometry.Envelope(o.id(),new Geometry.Rectangle(x-radius,y-radius,x+radius,y+radius),-1));
        }
        return new Geometry.MapSnapshot(width,height,RESOLUTION,cols,rows,new boolean[cols*rows],envelopes);
    }
    private static Geometry.Pose translate(Geometry.Pose p,double x,double y) { return new Geometry.Pose(p.x()+x,p.y()+y,p.headingRadians()); }
    private static MotionPath.Curve translate(MotionPath.Curve c,double x,double y,Runnable poll) {
        List<MotionPath.Bezier> segments=new ArrayList<>();
        for (MotionPath.Bezier b:c.segments()) { poll.run(); segments.add(new MotionPath.Bezier(translate(b.p0(),x,y),translate(b.p1(),x,y),translate(b.p2(),x,y),translate(b.p3(),x,y))); }
        List<MotionPath.Knot> knots=new ArrayList<>();
        for (MotionPath.Knot k:c.knots()) { poll.run(); knots.add(new MotionPath.Knot(k.distanceM(),k.segment(),k.u(),k.x()+x,k.y()+y,k.tangentRadians(),k.curvatureBound(),k.stop())); }
        return new MotionPath.Curve(segments,knots,c.lengthM(),c.detail());
    }
    private Proposal failure(PlannerBackend.Request r,PlannerBackend.Status status,long began,String detail) {
        long duration=Math.max(0,nanoTime.getAsLong()-began);
        return new Proposal(new PlannerBackend.Result(r.requestId(),r.taskId(),r.epoch(),r.snapshotId(),r.obstacleMapVersion(),status,null,null,
                duration,r.validUntilUs(),BACKEND_ID,detail==null ? "Motion planning rejected" : detail),null,null,0,0,0,duration);
    }
    private static final class Selection {
        MotionPath.Curve curve; MotionPath.Profile profile; PlannerBackend.Result seed; int count;
        final Set<List<MotionPath.Bezier>> seen=new HashSet<>();
        String rejection="No collision-certified curve with a feasible motion profile";
    }
    private static final class Cancelled extends RuntimeException { private static final long serialVersionUID=1L; }
    private static final class Expired extends RuntimeException { private static final long serialVersionUID=1L; }
    private static final class RefineDone extends RuntimeException { private static final long serialVersionUID=1L; }
}
