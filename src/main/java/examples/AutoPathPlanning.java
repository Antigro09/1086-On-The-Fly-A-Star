package pathplanning.examples;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import pathplanning.core.AStar;
import pathplanning.core.PathSmoother;
import pathplanning.core.VisibilityGraph;
import pathplanning.dynamic.DynamicObstacleManager;
import pathplanning.integration.NetworkTablesBridge;
import pathplanning.integration.TrajectoryPublisher;
import pathplanning.integration.VisionDataReceiver;
import pathplanning.optimization.PathOptimizer;
import pathplanning.optimization.VelocityProfiler;
import pathplanning.util.ConfigManager;
import pathplanning.util.FieldMap;
import pathplanning.util.Path;

import java.util.List;

/**
 * Example: Autonomous path planning during auto period
 */
public class AutoPathPlanning {
    
    public static void main(String[] args) {
        // Load configuration
        ConfigManager config = new ConfigManager("config/planner_params.yaml");
        
        // Initialize field map
        FieldMap fieldMap = new FieldMap("config/field_config.json");
        
        // Initialize A* pathfinder
        AStar pathfinder = new AStar(
            fieldMap.getStaticObstacles(),
            config.getDouble("planning.heuristic_weight", 1.0)
        );
        
        // Initialize dynamic obstacle manager
        DynamicObstacleManager obstacleManager = new DynamicObstacleManager(
            config.getDouble("robot.radius", 0.4),
            config.getDouble("obstacles.prediction_time", 1.0),
            config.getInt("obstacles.expiry_time_ms", 2000)
        );
        
        // Initialize vision receiver
        VisionDataReceiver visionReceiver = new VisionDataReceiver("vision");
        
        // Initialize network communication
        NetworkTablesBridge bridge = new NetworkTablesBridge("pathplanner");
        TrajectoryPublisher publisher = new TrajectoryPublisher("pathplanner");
        
        // Initialize optimizers
        PathOptimizer pathOptimizer = new PathOptimizer(
            config.getDouble("optimization.max_curvature", 2.0),
            config.getDouble("robot.max_velocity", 3.0),
            config.getDouble("robot.max_acceleration", 2.0)
        );
        
        VelocityProfiler profiler = new VelocityProfiler(
            config.getDouble("robot.max_velocity", 3.0),
            config.getDouble("robot.max_acceleration", 2.0),
            config.getDouble("robot.max_angular_velocity", Math.PI)
        );
        
        // Example auto sequence: Start -> Score Position 1 -> Pickup -> Score Position 2
        Pose2d[] autoWaypoints = {
            new Pose2d(1.5, 3.0, Rotation2d.fromDegrees(0)),    // Starting position
            new Pose2d(14.0, 4.5, Rotation2d.fromDegrees(180)), // Score position 1
            new Pose2d(7.0, 3.0, Rotation2d.fromDegrees(0)),    // Pickup location
            new Pose2d(14.0, 2.5, Rotation2d.fromDegrees(180))  // Score position 2
        };
        
        System.out.println("Starting autonomous path planning...");
        
        // Plan paths for each segment
        for (int i = 0; i < autoWaypoints.length - 1; i++) {
            Pose2d start = autoWaypoints[i];
            Pose2d goal = autoWaypoints[i + 1];
            
            System.out.println("\nPlanning segment " + (i + 1) + ":");
            System.out.println("  Start: " + start);
            System.out.println("  Goal: " + goal);
            
            long startTime = System.currentTimeMillis();
            
            // Update obstacles from vision
            List<VisionDataReceiver.DetectedObject> detections = visionReceiver.getLatestObjects();
            obstacleManager.updateDynamicObstacles(detections);
            
            // Find path
            Path path = pathfinder.findPath(
                start,
                goal,
                obstacleManager.getDynamicObstacles()
            );
            
            if (path == null) {
                System.err.println("  Failed to find path!");
                bridge.setPathStatus("failed");
                continue;
            }
            
            System.out.println("  Raw path: " + path.size() + " waypoints");
            
            // Optimize path
            if (config.getBoolean("optimization.smoothing_enabled", true)) {
                path = PathSmoother.simplify(path, fieldMap.getStaticObstacles());
                path = PathSmoother.bezierSmooth(
                    path,
                    config.getInt("optimization.bezier_points_per_segment", 10)
                );
                path = pathOptimizer.constrainToDynamics(path);
            }
            
            System.out.println("  Optimized path: " + path.size() + " waypoints");
            
            // Generate velocity profile
            VelocityProfiler.Trajectory trajectory = profiler.generateProfile(path, start);
            
            long planningTime = System.currentTimeMillis() - startTime;
            System.out.println("  Planning time: " + planningTime + " ms");
            System.out.println("  Trajectory duration: " + 
                String.format("%.2f", trajectory.getTotalTime()) + " s");
            
            // Publish trajectory
            bridge.publishTrajectory(trajectory);
            publisher.publishForVisualization(trajectory);
            bridge.publishPlanningTime(planningTime);
            
            System.out.println("  Path published successfully!");
            
            // Wait for robot to complete segment
            try {
                Thread.sleep((long) (trajectory.getTotalTime() * 1000));
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        
        System.out.println("\nAutonomous path planning complete!");
    }
}