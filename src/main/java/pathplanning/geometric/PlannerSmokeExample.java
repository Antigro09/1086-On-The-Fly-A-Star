package pathplanning.geometric;

import java.util.List;
import pathplanning.geometric.Geometry.*;

/** CPU-only synthetic obstacle example; no robot, NetworkTables or controller interaction. */
public final class PlannerSmokeExample {
    public static void main(String[] args) {
        MapSnapshot map=new MapSnapshot(6,4,0.1,60,40,new boolean[2400],
            List.of(new Envelope("synthetic-block",new Rectangle(2.5,0.7,3.5,2.9),0)));
        Input input=new Input(new Pose(0.8,1.8,0),new Pose(5.2,1.8,Math.PI/2),new Footprint(0.5,0.5,0.02),
            new Constraints(2,2,3,0,0,0.02,0.05),map,new Budget(500_000_000L,50_000,50_000,50_000),true);
        Output result=new GeometricSolver().plan(input,()->false);
        System.out.println(result.status()+"; points="+result.poses().size()+"; solver elapsed ns="+result.planningDurationNanos());
        if(result.status()!=Status.SUCCESS) throw new IllegalStateException(result.detail());
        for(Pose pose:result.poses()) System.out.println(pose);
        System.out.println("Proposed geometry only. No motion duration, execution or physical objective completion.");
    }
}
