package pathplanning.integration;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import pathplanning.optimization.VelocityProfiler.Trajectory;
import pathplanning.optimization.VelocityProfiler.TrajectoryState;

/**
 * Publishes trajectories in various formats for visualization and following
 */
public class TrajectoryPublisher {
    private final NetworkTable table;
    
    public TrajectoryPublisher(String tableName) {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        this.table = inst.getTable(tableName);
    }
    
    /**
     * Publish trajectory for PathPlanner visualization
     */
    public void publishForVisualization(Trajectory trajectory) {
        if (trajectory == null) {
            return;
        }
        
        // Publish all states for visualization
        int numStates = trajectory.getStates().size();
        
        double[] timestamps = new double[numStates];
        double[] xPositions = new double[numStates];
        double[] yPositions = new double[numStates];
        double[] headings = new double[numStates];
        double[] velocities = new double[numStates];
        
        for (int i = 0; i < numStates; i++) {
            TrajectoryState state = trajectory.getStates().get(i);
            timestamps[i] = state.getTime();
            xPositions[i] = state.getPose().getX();
            yPositions[i] = state.getPose().getY();
            headings[i] = state.getPose().getRotation().getRadians();
            velocities[i] = state.getVelocity();
        }
        
        table.getEntry("trajectory/timestamps").setDoubleArray(timestamps);
        table.getEntry("trajectory/x").setDoubleArray(xPositions);
        table.getEntry("trajectory/y").setDoubleArray(yPositions);
        table.getEntry("trajectory/headings").setDoubleArray(headings);
        table.getEntry("trajectory/velocities").setDoubleArray(velocities);
        table.getEntry("trajectory/valid").setBoolean(true);
    }
    
    /**
     * Publish trajectory in compact format for robot following
     */
    public void publishCompact(Trajectory trajectory) {
        if (trajectory == null) {
            table.getEntry("compact_trajectory").setDoubleArray(new double[0]);
            return;
        }
        
        double[] compact = trajectory.toDoubleArray();
        table.getEntry("compact_trajectory").setDoubleArray(compact);
        table.getEntry("total_time").setDouble(trajectory.getTotalTime());
    }
    
    /**
     * Clear published trajectory
     */
    public void clear() {
        table.getEntry("trajectory/valid").setBoolean(false);
        table.getEntry("compact_trajectory").setDoubleArray(new double[0]);
    }
}