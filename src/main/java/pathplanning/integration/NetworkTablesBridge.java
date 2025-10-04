package pathplanning.integration;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import pathplanning.optimization.VelocityProfiler.Trajectory;

/**
 * Bridge for communication between Jetson and roboRIO via NetworkTables
 */
public class NetworkTablesBridge {
    private final NetworkTable table;
    
    private static final String ROBOT_POSE_KEY = "robot_pose";
    private static final String TARGET_POSE_KEY = "target_pose";
    private static final String TRAJECTORY_KEY = "trajectory";
    private static final String TRAJECTORY_TIMESTAMP_KEY = "trajectory_timestamp";
    private static final String TRAJECTORY_LENGTH_KEY = "trajectory_length";
    private static final String PATH_STATUS_KEY = "path_status";
    private static final String PLANNING_TIME_KEY = "planning_time_ms";
    
    public NetworkTablesBridge(String tableName) {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        this.table = inst.getTable(tableName);
        
        // Set default values
        table.getEntry(PATH_STATUS_KEY).setString("idle");
        table.getEntry(PLANNING_TIME_KEY).setDouble(0);
    }
    
    /**
     * Publish trajectory to roboRIO
     */
    public void publishTrajectory(Trajectory trajectory) {
        if (trajectory == null) {
            table.getEntry(PATH_STATUS_KEY).setString("no_path");
            return;
        }
        
        // Serialize trajectory to double array
        double[] states = trajectory.toDoubleArray();
        table.getEntry(TRAJECTORY_KEY).setDoubleArray(states);
        
        // Publish metadata
        table.getEntry(TRAJECTORY_TIMESTAMP_KEY).setDouble(
            System.currentTimeMillis() / 1000.0
        );
        table.getEntry(TRAJECTORY_LENGTH_KEY).setDouble(trajectory.getTotalTime());
        table.getEntry(PATH_STATUS_KEY).setString("ready");
    }
    
    /**
     * Get current robot pose from roboRIO
     */
    public Pose2d getRobotPose() {
        double[] pose = table.getEntry(ROBOT_POSE_KEY).getDoubleArray(new double[]{0, 0, 0});
        
        if (pose.length >= 3) {
            return new Pose2d(pose[0], pose[1], new Rotation2d(pose[2]));
        }
        
        return new Pose2d();
    }
    
    /**
     * Get target pose from roboRIO
     */
    public Pose2d getTargetPose() {
    double[] target = table.getEntry(TARGET_POSE_KEY).getDoubleArray(new double[0]);
        
        if (target != null && target.length >= 3) {
            return new Pose2d(target[0], target[1], new Rotation2d(target[2]));
        }
        
        return null;
    }
    
    /**
     * Set target pose (for roboRIO to command path planning)
     */
    public void setTargetPose(Pose2d target) {
        if (target != null) {
            double[] pose = {
                target.getX(),
                target.getY(),
                target.getRotation().getRadians()
            };
            table.getEntry(TARGET_POSE_KEY).setDoubleArray(pose);
        } else {
            table.getEntry(TARGET_POSE_KEY).setDoubleArray(new double[0]);
        }
    }
    
    /**
     * Publish planning statistics
     */
    public void publishPlanningTime(double timeMs) {
        table.getEntry(PLANNING_TIME_KEY).setDouble(timeMs);
    }
    
    /**
     * Set path status
     */
    public void setPathStatus(String status) {
        table.getEntry(PATH_STATUS_KEY).setString(status);
    }
    
    /**
     * Get path status
     */
    public String getPathStatus() {
        return table.getEntry(PATH_STATUS_KEY).getString("idle");
    }
    
    /**
     * Check if new target is available
     */
    public boolean hasNewTarget() {
    double[] target = table.getEntry(TARGET_POSE_KEY).getDoubleArray(new double[0]);
        return target != null && target.length >= 3;
    }
    
    /**
     * Clear current target
     */
    public void clearTarget() {
        table.getEntry(TARGET_POSE_KEY).setDoubleArray(new double[0]);
    }
}