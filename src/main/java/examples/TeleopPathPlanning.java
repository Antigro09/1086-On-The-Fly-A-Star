package pathplanning.examples;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import pathplanning.core.AStar;
import pathplanning.core.PathSmoother;
import pathplanning.core.VisibilityGraph;
import pathplanning.dynamic.DynamicObstacleManager;
import pathplanning.dynamic.RealTimeReplanner;
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
 * Example: Teleoperated path planning with real-time replanning
 */
public class TeleopPathPlanning {
    
    private static Path currentPath = null;
    
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
        
        // Initialize replanner
        RealTimeReplanner replanner = new RealTimeReplanner(
            config.getInt("planning.frequency_hz", 20),
            config.getDouble("planning.deviation_threshold", 0.3),
            0.2
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
        
        System.out.println("Starting teleoperated path planning service...");
        System.out.println("Waiting for target poses from driver station...");
        
        // Main teleoperation loop
        int loopCount = 0;
        while (true) {
            loopCount++;
            
            try {
                // Get current robot pose
                Pose2d currentPose = bridge.getRobotPose();
                
                // Check for new target from driver
                Pose2d targetPose = bridge.getTargetPose();
                
                if (targetPose != null) {
                    // Update obstacles from vision
                    List<VisionDataReceiver.DetectedObject> detections = 
                        visionReceiver.getLatestObjects();
                    obstacleManager.updateDynamicObstacles(detections);
                    
                    // Check if replanning is needed
                    boolean shouldReplan = replanner.shouldReplan(
                        currentPose,
                        currentPath,
                        obstacleManager
                    );
                    
                    if (shouldReplan) {
                        if (loopCount % 20 == 0) { // Print every second
                            System.out.println("\nReplanning triggered:");
                            System.out.println("  Current pose: " + currentPose);
                            System.out.println("  Target pose: " + targetPose);
                            System.out.println("  Obstacles: " + 
                                obstacleManager.getObstacleCount());
                        }
                        
                        long startTime = System.currentTimeMillis();
                        
                        // Find new path
                        Path path = pathfinder.findPath(
                            currentPose,
                            targetPose,
                            obstacleManager.getDynamicObstacles()
                        );
                        
                        if (path != null) {
                            // Optimize path
                            if (config.getBoolean("optimization.smoothing_enabled", true)) {
                                path = PathSmoother.simplify(
                                    path,
                                    fieldMap.getStaticObstacles()
                                );
                                path = PathSmoother.bezierSmooth(
                                    path,
                                    config.getInt("optimization.bezier_points_per_segment", 10)
                                );
                                path = pathOptimizer.constrainToDynamics(path);
                            }
                            
                            // Generate velocity profile
                            VelocityProfiler.Trajectory trajectory = 
                                profiler.generateProfile(path, currentPose);
                            
                            // Publish trajectory
                            bridge.publishTrajectory(trajectory);
                            publisher.publishForVisualization(trajectory);
                            
                            long planningTime = System.currentTimeMillis() - startTime;
                            bridge.publishPlanningTime(planningTime);
                            bridge.setPathStatus("following");
                            
                            currentPath = path;
                            
                            if (loopCount % 20 == 0) {
                                System.out.println("  Planning time: " + planningTime + " ms");
                                System.out.println("  Path length: " + 
                                    String.format("%.2f m", path.getTotalLength()));
                            }
                        } else {
                            System.err.println("  Failed to find path!");
                            bridge.setPathStatus("no_path");
                            currentPath = null;
                        }
                    }
                } else {
                    // No target, clear path
                    if (currentPath != null) {
                        currentPath = null;
                        bridge.setPathStatus("idle");
                        publisher.clear();
                    }
                }
                
                // Sleep to maintain loop rate
                int sleepMs = 1000 / config.getInt("planning.frequency_hz", 20);
                Thread.sleep(sleepMs);
                
            } catch (InterruptedException e) {
                System.err.println("Planning loop interrupted!");
                break;
            } catch (Exception e) {
                System.err.println("Error in planning loop: " + e.getMessage());
                e.printStackTrace();
            }
        }
        
        System.out.println("Path planning service stopped.");
    }
}