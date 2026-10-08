package pathplanning.benchmark;

import java.lang.management.ManagementFactory;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.security.MessageDigest;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.Future;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;
import org.frcworldstate.core.Geometry.*;
import org.frcworldstate.core.PlannerBackend;
import pathplanning.backend.AStarPlannerBackend;
import pathplanning.geometric.GeometricSolver;
import pathplanning.geometric.Geometry;

/** Small CPU-only measurement tool, not a controller benchmark or motion executor. */
public final class PlannerBenchmark {
    private record Obstacle(String id,double x,double y,double radius,double margin,boolean dynamic) {}
    private record Fixture(String name,List<Obstacle> obstacles,double startY,double goalY,boolean cancel) {}
    private record Sample(long endToEnd,long work,long reported,long cpu,long allocated,long cancelUnwind,
                          int expanded,int points,String status,String detail,int cancellationPolls) {}
    private static final long ROBOT_ORIGIN_US=1_000_000;
    private static final long HOST_ORIGIN_NS=System.nanoTime();
    private static final int CANCEL_AFTER_POLLS=256;
    private static long robotTimeUs() { return ROBOT_ORIGIN_US+(System.nanoTime()-HOST_ORIGIN_NS)/1000; }
    private static final class Probe implements PlannerBackend.Cancellation {
        final boolean enabled; int calls; long triggered=-1;
        Probe(boolean enabled) { this.enabled=enabled; }
        public boolean cancelled() {
            calls++;
            if(enabled&&calls>=CANCEL_AFTER_POLLS) {
                if(triggered<0) triggered=System.nanoTime();
                return true;
            }
            return false;
        }
    }
    private static final class Metrics {
        final java.lang.management.ThreadMXBean thread=ManagementFactory.getThreadMXBean();
        final com.sun.management.ThreadMXBean allocation;
        Metrics() {
            if(thread.isThreadCpuTimeSupported()&&!thread.isThreadCpuTimeEnabled()) thread.setThreadCpuTimeEnabled(true);
            allocation=thread instanceof com.sun.management.ThreadMXBean m&&m.isThreadAllocatedMemorySupported()?m:null;
            if(allocation!=null&&!allocation.isThreadAllocatedMemoryEnabled()) allocation.setThreadAllocatedMemoryEnabled(true);
        }
        long allocated() { return allocation==null?-1:allocation.getThreadAllocatedBytes(Thread.currentThread().getId()); }
        long cpu() { return thread.isThreadCpuTimeSupported()?thread.getCurrentThreadCpuTime():-1; }
    }
    private final double width,height;
    private final long limitNs;
    private final AStarPlannerBackend backend=new AStarPlannerBackend(PlannerBenchmark::robotTimeUs);
    private final GeometricSolver solver=new GeometricSolver();
    private final Metrics metrics=new Metrics();
    private PlannerBenchmark(double width,double height,long limitNs) { this.width=width;this.height=height;this.limitNs=limitNs; }
    private List<Fixture> fixtures() {
        List<Fixture> all=new ArrayList<>();
        all.add(new Fixture("open",List.of(),height*.25,height*.75,false));
        List<Obstacle> clutter=new ArrayList<>();
        for(int x=1;x<=4;x++) for(int y=1;y<=3;y++)
            clutter.add(new Obstacle("clutter-"+x+"-"+y,width*x/5,height*y/4,.22,0,false));
        all.add(new Fixture("cluttered",clutter,height*.25,height*.75,false));
        List<Obstacle> wall=new ArrayList<>(),narrow=new ArrayList<>();
        for(int y=0;y<=Math.ceil(height/.4);y++) {
            double center=y*.4;
            Obstacle o=new Obstacle("wall-"+y,width/2,center,.23,0,false);wall.add(o);
            if(Math.abs(center-height/2)>.43) narrow.add(o);
        }
        all.add(new Fixture("narrow-passage",narrow,height*.2,height*.2,false));
        all.add(new Fixture("no-path",wall,height*.25,height*.75,false));
        // 0.1m uncertainty + 1m/s * (0.2s observation age + 0.2s request horizon).
        all.add(new Fixture("moving-envelope",List.of(new Obstacle("moving",width/2,height/2,.3,.5,true)),height*.25,height*.75,false));
        all.add(new Fixture("cancellation",wall,height*.25,height*.75,true));
        return all;
    }
    private PlannerBackend.Request request(Fixture fixture,Probe probe) {
        long nowUs=robotTimeUs();long started=System.nanoTime();
        List<PlannerBackend.Obstacle> obstacles=new ArrayList<>();
        for(Obstacle o:fixture.obstacles) obstacles.add(new PlannerBackend.Obstacle(o.id,new Vec2(o.x,o.y),o.radius,o.margin,nowUs+200_000,o.dynamic));
        return new PlannerBackend.Request("benchmark","offline",1,1,1,new FieldIdentity("generic","synthetic-benchmark","1"),
            new Pose2(new Vec2(.6,fixture.startY),0),Velocity2.zero(),new PlannerBackend.Goal(new Pose2(new Vec2(width-.6,fixture.goalY),Math.PI/2),.01,.03,.03),
            new PlannerBackend.Footprint(.4,.4),new PlannerBackend.Constraints(3,3,4),new PlannerBackend.Bounds(0,0,width,height),obstacles,nowUs,nowUs+200_000,
            new PlannerBackend.SearchBudget(started,limitNs),probe);
    }
    private Geometry.Input coreInput(PlannerBackend.Request request) {
        List<Geometry.Envelope> envelopes=new ArrayList<>();
        for(PlannerBackend.Obstacle o:request.obstacles()) {
            double r=o.envelopeRadiusM(); envelopes.add(new Geometry.Envelope(o.id(),new Geometry.Rectangle(o.centerM().x()-r,o.centerM().y()-r,o.centerM().x()+r,o.centerM().y()+r),-1));
        }
        int columns=(int)Math.ceil(width/.1),rows=(int)Math.ceil(height/.1);
        Geometry.MapSnapshot map=new Geometry.MapSnapshot(width,height,.1,columns,rows,new boolean[columns*rows],envelopes);
        return new Geometry.Input(new Geometry.Pose(.6,request.startPose().position().y(),0),new Geometry.Pose(width-.6,request.goal().pose().position().y(),Math.PI/2),
            new Geometry.Footprint(.4,.4,.02),new Geometry.Constraints(3,3,4,0,0,.01,.03),map,new Geometry.Budget(limitNs,100_000,250_000,250_000),true);
    }
    private Sample execute(Fixture fixture,boolean core) {
        Probe probe=new Probe(fixture.cancel);long a0=metrics.allocated(),c0=metrics.cpu(),began=System.nanoTime();
        PlannerBackend.Request request=request(fixture,probe);
        long reported;int expanded=-1,points;String status,detail;
        if(core) {
            Geometry.Output result=solver.plan(coreInput(request),probe::cancelled);
            reported=result.planningDurationNanos();expanded=result.expandedNodes();points=result.poses().size();status=result.status().name();detail=result.detail();
        } else {
            PlannerBackend.Result result=backend.plan(request);
            reported=result.solverDurationNanos();points=result.path()==null?0:result.path().points().size();status=result.status().name();detail=result.detail();
        }
        long ended=System.nanoTime(),cpu=metrics.cpu(),allocated=metrics.allocated();
        // A genuine deadline can win before poll-count cancellation; retain that typed outcome.
        if(fixture.cancel&&!status.equals("CANCELLED")&&!status.equals("TIMEOUT"))
            throw new IllegalStateException("Cancellation fixture did not stop: "+status);
        return new Sample(0,ended-began,reported,c0<0||cpu<0?-1:cpu-c0,a0<0||allocated<0?-1:allocated-a0,
            probe.triggered<0?-1:ended-probe.triggered,expanded,points,status,detail,probe.calls);
    }
    private static long percentile(long[] source,double quantile) {
        long[] sorted=source.clone();Arrays.sort(sorted);return sorted[Math.max(0,(int)Math.ceil(quantile*sorted.length)-1)];
    }
    private static String quote(String s) { return "\""+s.replace("\\","\\\\").replace("\"","\\\"").replace("\n","\\n").replace("\r","\\r")+"\""; }
    private static String sha256(String value) throws Exception {
        return java.util.HexFormat.of().formatHex(MessageDigest.getInstance("SHA-256").digest(value.getBytes(StandardCharsets.UTF_8)));
    }
    private static String distribution(long[] values) {
        if(values.length==0||values[0]<0) return "null";
        return String.format(Locale.ROOT,"{\"p50\":%d,\"p95\":%d,\"p99\":%d,\"max\":%d}",percentile(values,.5),percentile(values,.95),percentile(values,.99),percentile(values,1));
    }
    private static long gcCount() { return ManagementFactory.getGarbageCollectorMXBeans().stream().mapToLong(b->Math.max(0,b.getCollectionCount())).sum(); }
    private static long gcMillis() { return ManagementFactory.getGarbageCollectorMXBeans().stream().mapToLong(b->Math.max(0,b.getCollectionTime())).sum(); }
    public static void main(String[] args) throws Exception {
        int warmup=100,samples=500;double width=8,height=4,deadlineMs=10,timeoutMs=100;String phase="adapter";Path out=Path.of("build/benchmarks/current");
        for(int i=0;i<args.length;i+=2) {
            if(i+1>=args.length) throw new IllegalArgumentException("Each option requires a value");
            switch(args[i]) {
                case "--warmup" -> warmup=Integer.parseInt(args[i+1]);case "--samples" -> samples=Integer.parseInt(args[i+1]);
                case "--width-m" -> width=Double.parseDouble(args[i+1]);case "--height-m" -> height=Double.parseDouble(args[i+1]);
                case "--deadline-ms" -> deadlineMs=Double.parseDouble(args[i+1]);case "--timeout-ms" -> timeoutMs=Double.parseDouble(args[i+1]);
                case "--phase" -> phase=args[i+1];case "--out" -> out=Path.of(args[i+1]);default -> throw new IllegalArgumentException("Unknown option "+args[i]);
            }
        }
        if(warmup<0||warmup>2000||samples<1||samples>5000||!Double.isFinite(width)||!Double.isFinite(height)||width<4||width>20||height<3||height>10||
            !Double.isFinite(deadlineMs)||!Double.isFinite(timeoutMs)||deadlineMs<=0||deadlineMs>1000||timeoutMs<=0||timeoutMs>1000||!(phase.equals("adapter")||phase.equals("core"))) throw new IllegalArgumentException("Invalid/beyond bounded benchmark options");
        Files.createDirectories(out);PlannerBenchmark benchmark=new PlannerBenchmark(width,height,(long)(timeoutMs*1e6));boolean core=phase.equals("core");
        ThreadPoolExecutor worker=new ThreadPoolExecutor(1,1,0,TimeUnit.MILLISECONDS,new ArrayBlockingQueue<>(1),r->new Thread(r,"benchmark-planner"));
        StringBuilder csv=new StringBuilder("workload,index,end_to_end_ns,worker_wall_ns,reported_ns,worker_cpu_ns,worker_allocated_bytes,cancel_trigger_to_return_ns,expanded_nodes,path_points,status,cancellation_polls,detail\n");
        StringBuilder report=new StringBuilder("{\n\"version\":\"frc-planner-benchmark/1\",\"recorded_utc\":"+quote(Instant.now().toString())+",\"target\":\"development Mac; synthetic CPU only\",\"java\":"+quote(System.getProperty("java.runtime.version"))+",\"vm\":"+quote(System.getProperty("java.vm.name"))+",\"os\":"+quote(System.getProperty("os.name")+" "+System.getProperty("os.version")+" "+System.getProperty("os.arch"))+",\"available_processors\":"+Runtime.getRuntime().availableProcessors()+",\"max_heap_bytes\":"+Runtime.getRuntime().maxMemory()+",\"phase\":"+quote(phase)+",\"warmup_per_workload\":"+warmup+",\"samples_per_workload\":"+samples+",\"deadline_ms\":"+deadlineMs+",\"search_timeout_ms\":"+timeoutMs+",\"field_width_m\":"+width+",\"field_height_m\":"+height+",\"grid_resolution_m\":0.1,\"robot_footprint_m\":[0.4,0.4],\"clearance_m\":0.02,\"workers\":1,\"queue_capacity\":1,\"ambient_load_before\":"+ManagementFactory.getOperatingSystemMXBean().getSystemLoadAverage()+",\"workloads\":[\n");
        long gc0=gcCount(),gct0=gcMillis();int ordinal=0;long runStarted=System.nanoTime();
        final long deadlineNs=(long)(deadlineMs*1e6);
        try {
            for(Fixture fixture:benchmark.fixtures()) {
                for(int j=0;j<warmup;j++) {
                    if(System.nanoTime()-runStarted>240_000_000_000L) throw new IllegalStateException("Overall 240s benchmark cap reached; reduce warmup");
                    worker.submit(()->benchmark.execute(fixture,core)).get(2,TimeUnit.SECONDS);
                }
                List<Sample> results=new ArrayList<>();Map<String,Integer> statuses=new LinkedHashMap<>(),details=new LinkedHashMap<>();
                for(int j=0;j<samples;j++) {
                    if(System.nanoTime()-runStarted>240_000_000_000L) throw new IllegalStateException("Overall 240s benchmark cap reached; reduce samples");
                    long submitted=System.nanoTime();Future<Sample> future=worker.submit(()->benchmark.execute(fixture,core));Sample result=future.get(2,TimeUnit.SECONDS);long latency=System.nanoTime()-submitted;
                    Sample s=new Sample(latency,result.work,result.reported,result.cpu,result.allocated,result.cancelUnwind,result.expanded,result.points,result.status,result.detail,result.cancellationPolls);
                    results.add(s);statuses.merge(s.status,1,Integer::sum);details.merge(s.detail,1,Integer::sum);
                    csv.append(fixture.name).append(',').append(j).append(',').append(s.endToEnd).append(',').append(s.work).append(',').append(s.reported).append(',').append(s.cpu).append(',').append(s.allocated).append(',').append(s.cancelUnwind).append(',').append(s.expanded).append(',').append(s.points).append(',').append(s.status).append(',').append(s.cancellationPolls).append(',').append('"').append(s.detail.replace("\"","\"\"")).append('"').append('\n');
                }
                long[] latency=results.stream().mapToLong(Sample::endToEnd).toArray(),work=results.stream().mapToLong(Sample::work).toArray(),reported=results.stream().mapToLong(Sample::reported).toArray(),allocation=results.stream().mapToLong(Sample::allocated).toArray(),cpu=results.stream().mapToLong(Sample::cpu).toArray(),unwind=results.stream().mapToLong(Sample::cancelUnwind).toArray(),expanded=results.stream().mapToLong(Sample::expanded).toArray();
                long misses=results.stream().filter(s->s.endToEnd>deadlineNs).count();
                String statusJson=statuses.entrySet().stream().map(e->quote(e.getKey())+":"+e.getValue()).reduce((a,b)->a+","+b).orElse("");
                String detailJson=details.entrySet().stream().map(e->quote(e.getKey())+":"+e.getValue()).reduce((a,b)->a+","+b).orElse("");
                if(ordinal++>0) report.append(",\n");
                report.append("{\"name\":").append(quote(fixture.name)).append(",\"fixture_sha256\":").append(quote(sha256(fixture.toString()+width+":"+height))).append(",\"obstacles\":").append(fixture.obstacles.size()).append(",\"statuses\":{").append(statusJson).append("},\"deadline_misses\":").append(misses).append(",\"end_to_end_ns\":").append(distribution(latency)).append(",\"worker_wall_ns\":").append(distribution(work)).append(",\"reported_ns\":").append(distribution(reported)).append(",\"worker_cpu_ns\":").append(distribution(cpu)).append(",\"worker_allocated_bytes\":").append(distribution(allocation)).append(",\"cancel_trigger_to_return_ns\":").append(distribution(unwind)).append(",\"expanded_nodes_separate_core_invocation\":").append(distribution(expanded)).append(",\"detail_counts\":{").append(detailJson).append("},\"last_detail\":").append(quote(results.get(results.size()-1).detail)).append('}');
                System.out.printf(Locale.ROOT,"%s/%s: p50 %.3f ms p95 %.3f p99 %.3f max %.3f; >%.1fms=%d/%d; %s%n",phase,fixture.name,percentile(latency,.5)/1e6,percentile(latency,.95)/1e6,percentile(latency,.99)/1e6,percentile(latency,1)/1e6,deadlineMs,misses,samples,statuses);
            }
        } finally { worker.shutdownNow();if(!worker.awaitTermination(2,TimeUnit.SECONDS)) throw new IllegalStateException("Worker did not stop"); }
        report.append("\n],\"ambient_load_after\":").append(ManagementFactory.getOperatingSystemMXBean().getSystemLoadAverage()).append(",\"gc_collections_during_run\":").append(gcCount()-gc0).append(",\"gc_collection_ms_during_run\":").append(gcMillis()-gct0).append(",\"allocation_kind\":\"worker-thread allocated bytes, not retained heap or RSS\",\"robot_clock\":\"explicit virtual monotonic microsecond domain anchored to host elapsed time; synthetic only\",\"cancellation\":\"deterministic poll threshold256; trigger-to-return excludes external cancel scheduling\",\"hardware_verified\":false}\n");
        Files.writeString(out.resolve("samples.csv"),csv);Files.writeString(out.resolve("summary.json"),report);
    }
}
