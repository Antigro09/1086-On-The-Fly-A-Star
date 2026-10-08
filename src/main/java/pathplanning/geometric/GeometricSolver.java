package pathplanning.geometric;

import java.util.*;
import java.util.function.LongSupplier;
import pathplanning.geometric.Geometry.*;

/** Bounded holonomic A*: geometric proposals only, with no following or task completion authority. */
public final class GeometricSolver {
    private final LongSupplier nanoTime;
    public GeometricSolver() { this(System::nanoTime); }
    public GeometricSolver(LongSupplier nanoTime) { this.nanoTime = Objects.requireNonNull(nanoTime); }
    public Output plan(Input input, Cancellation cancellation) {
        long began=nanoTime.getAsLong();
        Run run=new Run(began,input,cancellation == null ? () -> false : cancellation);
        try {
            String invalid=validate(input);
            if (invalid != null) return run.fail(Status.INVALID_INPUT,invalid);
            run.poll();
            double radius=input.footprint().enclosingRadius();
            if (!SweptCollision.safe(input.map(),input.start(),input.start(),radius,run::poll))
                return run.fail(Status.UNSAFE_START,"Start footprint crosses occupancy or field boundary");
            if (!SweptCollision.safe(input.map(),input.goal(),input.goal(),radius + input.constraints().positionTolerance(),run::poll))
                return run.fail(Status.UNSAFE_GOAL,"Goal footprint or complete tolerance region is unsafe");
            List<Pose> path=search(run,radius);
            if (path == null) return run.fail(Status.NO_PATH,"No validated path in discretized search graph");
            if (!validatePath(input.map(),path,radius,run::poll)) return run.fail(Status.COLLISION_VALIDATION_FAILED,"Reconstruction failed collision check");
            if (input.simplify()) path=simplify(input.map(),path,radius,run::poll);
            if (!validatePath(input.map(),path,radius,run::poll)) return run.fail(Status.COLLISION_VALIDATION_FAILED,"Simplification failed collision check");
            run.poll();
            return new Output(Status.SUCCESS,path,run.elapsed(),run.expanded,"Geometric path; requires independent execution guard and time parameterization");
        } catch (Stopped stop) { return run.fail(stop.status,stop.getMessage()); }
    }
    public static String validate(Input i) {
        if (i==null || i.start()==null || i.goal()==null || i.map()==null || i.footprint()==null || i.constraints()==null || i.budget()==null) return "Missing input";
        if (!finite(i.start().x(),i.start().y(),i.start().headingRadians(),i.goal().x(),i.goal().y(),i.goal().headingRadians())) return "Non-finite pose";
        Footprint f=i.footprint(); Constraints c=i.constraints(); MapSnapshot m=i.map(); Budget b=i.budget();
        if (!finite(f.lengthMeters(),f.widthMeters(),f.clearanceMeters(),f.enclosingRadius()) || f.lengthMeters()<=0 || f.widthMeters()<=0 || f.clearanceMeters()<0) return "Invalid raw footprint";
        if (!finite(c.maxSpeed(),c.maxAcceleration(),c.maxAngularSpeed(),c.startSpeed(),c.endSpeed(),c.positionTolerance(),c.headingToleranceRadians()) ||
            c.maxSpeed()<=0 || c.maxAcceleration()<=0 || c.maxAngularSpeed()<=0 || c.startSpeed()<0 || c.endSpeed()<0 ||
            c.startSpeed()>c.maxSpeed() || c.endSpeed()>c.maxSpeed() || c.positionTolerance()<0 || c.headingToleranceRadians()<0 || c.headingToleranceRadians()>Math.PI) return "Invalid constraints";
        if (b.timeoutNanos()<=0 || b.timeoutNanos()>5_000_000_000L || b.maxExpanded()<=0 || b.maxExpanded()>1_000_000 ||
            b.maxQueueEntries()<=0 || b.maxQueueEntries()>1_000_000 || b.maxCells()<=0 || b.maxCells()>1_000_000) return "Invalid or excessive search budget";
        if (!finite(m.width(),m.height(),m.cellSize()) || m.width()<=0 || m.height()<=0 || m.width()>1000 || m.height()>1000 || m.cellSize()<0.001 ||
            m.columns()<=0 || m.rows()<=0 || (long)m.columns()*m.rows()>b.maxCells() || m.occupancySize()!=(long)m.columns()*m.rows() ||
            m.columns()!=Math.ceil(m.width()/m.cellSize()) || m.rows()!=Math.ceil(m.height()/m.cellSize()) || m.dynamic()==null || m.dynamic().size()>4096) return "Invalid map dimensions, cells or obstacle count";
        Set<String> ids=new HashSet<>();
        for (Envelope e:m.dynamic()) {
            if (e==null || e.objectId()==null || e.objectId().isBlank() || !ids.add(e.objectId()) || e.bounds()==null || e.observationAgeMicros() < -1) return "Invalid or duplicate obstacle identity/age";
            Rectangle r=e.bounds();
            if (!finite(r.minX(),r.minY(),r.maxX(),r.maxY()) || Math.abs(r.minX())>10000 || Math.abs(r.minY())>10000 ||
                Math.abs(r.maxX())>10000 || Math.abs(r.maxY())>10000 || r.maxX()<=r.minX() || r.maxY()<=r.minY()) return "Invalid or uncomputable obstacle envelope";
        }
        return null;
    }
    private static boolean finite(double... values) { for(double value:values) if(!Double.isFinite(value)) return false; return true; }
    private List<Pose> search(Run run,double radius) {
        Input i=run.input; MapSnapshot map=i.map(); Pose start=i.start(),goal=i.goal();
        if (SweptCollision.safe(map,start,goal,radius,run::poll)) return endpointPath(start,goal);
        int count=map.occupancySize(); double[] distance=new double[count]; Arrays.fill(distance,Double.POSITIVE_INFINITY);
        int[] parent=new int[count]; Arrays.fill(parent,-1); boolean[] closed=new boolean[count];
        PriorityQueue<Node> queue=new PriorityQueue<>(Comparator.comparingDouble(Node::score).thenComparingInt(Node::index));
        int sx=(int)Math.floor(start.x()/map.cellSize()),sy=(int)Math.floor(start.y()/map.cellSize());
        for(int y=Math.max(0,sy-1);y<=Math.min(map.rows()-1,sy+1);y++) for(int x=Math.max(0,sx-1);x<=Math.min(map.columns()-1,sx+1);x++) {
            Pose p=center(map,x,y,start.headingRadians());
            if(SweptCollision.safe(map,start,p,radius,run::poll)) {
                int index=y*map.columns()+x; distance[index]=distance(start,p); enqueue(run,queue,new Node(index,distance[index],distance[index]+distance(p,goal)));
            }
        }
        while(!queue.isEmpty()) {
            run.poll(); Node node=queue.remove(); int index=node.index();
            if(closed[index] || node.cost()!=distance[index]) continue;
            if(run.expanded>=i.budget().maxExpanded()) throw new Stopped(Status.RESOURCE_LIMIT,"Expansion budget exceeded");
            closed[index]=true; run.expanded++;
            int x=index%map.columns(),y=index/map.columns(); Pose p=center(map,x,y,start.headingRadians());
            // Goal connection is checked at full footprint; goal tolerance never bypasses collision validation.
            if(SweptCollision.safe(map,p,goal,radius,run::poll)) {
                List<Pose> reversed=new ArrayList<>(); int at=index;
                while(at>=0) { run.poll(); reversed.add(center(map,at%map.columns(),at/map.columns(),start.headingRadians())); at=parent[at]; }
                Collections.reverse(reversed); List<Pose> path=new ArrayList<>(); path.add(start); path.addAll(reversed);
                appendGoal(path,goal); return path;
            }
            for(int dy=-1;dy<=1;dy++) for(int dx=-1;dx<=1;dx++) {
                if(dx==0 && dy==0) continue; int nx=x+dx,ny=y+dy;
                if(nx<0 || ny<0 || nx>=map.columns() || ny>=map.rows()) continue;
                int next=ny*map.columns()+nx; if(closed[next] || map.occupied(nx,ny)) continue;
                Pose q=center(map,nx,ny,start.headingRadians()); double candidate=distance[index]+distance(p,q);
                if(candidate>=distance[next] || !SweptCollision.safe(map,p,q,radius,run::poll)) continue;
                distance[next]=candidate; parent[next]=index;
                enqueue(run,queue,new Node(next,candidate,candidate+distance(q,goal)));
            }
        }
        return null;
    }
    private static void enqueue(Run run,PriorityQueue<Node> queue,Node node) {
        if(queue.size()>=run.input.budget().maxQueueEntries()) throw new Stopped(Status.RESOURCE_LIMIT,"Queue budget exceeded"); queue.add(node);
    }
    private static Pose center(MapSnapshot map,int x,int y,double heading) {
        return new Pose(Math.min(map.width(),(x+0.5)*map.cellSize()),Math.min(map.height(),(y+0.5)*map.cellSize()),heading);
    }
    private static List<Pose> endpointPath(Pose start,Pose goal) { List<Pose> list=new ArrayList<>(); list.add(start); appendGoal(list,goal); return list; }
    private static void appendGoal(List<Pose> path,Pose goal) {
        Pose last=path.get(path.size()-1);
        if(distance(last,goal)>0) path.add(new Pose(goal.x(),goal.y(),last.headingRadians()));
        last=path.get(path.size()-1);
        if(!last.equals(goal)) path.add(goal);
    }
    private static double distance(Pose a,Pose b) { return Math.hypot(a.x()-b.x(),a.y()-b.y()); }
    public static boolean validatePath(MapSnapshot map,List<Pose> poses,double radius,Runnable poll) {
        if(poses.isEmpty()) return false;
        for(int at=0;at<poses.size();at++) {
            Pose p=poses.get(at); if(!finite(p.x(),p.y(),p.headingRadians())) return false;
            if(!SweptCollision.safe(map,at==0?p:poses.get(at-1),p,radius,poll)) return false;
        }
        return true;
    }
    public static List<Pose> simplify(MapSnapshot map,List<Pose> path,double radius,Runnable poll) {
        List<Pose> output=new ArrayList<>(); int at=0; output.add(path.get(0));
        while(at<path.size()-1) {
            int next=at+1;
            for(int candidate=path.size()-1;candidate>at+1;candidate--) {
                poll.run();
                // Preserve deliberate final in-place rotation and holonomic heading.
                if(path.get(at).headingRadians()==path.get(candidate).headingRadians() && SweptCollision.safe(map,path.get(at),path.get(candidate),radius,poll)) { next=candidate; break; }
            }
            output.add(path.get(next)); at=next;
        }
        return List.copyOf(output);
    }
    private record Node(int index,double cost,double score) {}
    private static final class Stopped extends RuntimeException {
        private static final long serialVersionUID = 1L;
        final Status status; Stopped(Status status,String message) { super(message,null,false,false); this.status=status; }
    }
    private final class Run {
        final long began; final Input input; final Cancellation cancellation; int expanded;
        Run(long began,Input input,Cancellation cancellation) { this.began=began;this.input=input;this.cancellation=cancellation; }
        long elapsed() { return Math.max(0,nanoTime.getAsLong()-began); }
        void poll() {
            if(cancellation.cancelled() || Thread.currentThread().isInterrupted()) throw new Stopped(Status.CANCELLED,"Cancelled before validated publication");
            if(elapsed()>=input.budget().timeoutNanos()) throw new Stopped(Status.TIMEOUT,"Monotonic search/validation budget exhausted");
        }
        Output fail(Status status,String detail) { return new Output(status,List.of(),elapsed(),expanded,detail); }
    }
}
