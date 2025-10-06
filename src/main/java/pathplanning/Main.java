package pathplanning;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import pathplanning.core.AStar;
import pathplanning.core.PathSmoother;
import pathplanning.dynamic.DynamicObstacleManager;
import pathplanning.dynamic.RealTimeReplanner;
import pathplanning.integration.NetworkTablesBridge;
import pathplanning.integration.TrajectoryPublisher;
import pathplanning.integration.VisionDataReceiver;
import pathplanning.optimization.PathOptimizer;
import pathplanning.optimization.VelocityProfiler;
import pathplanning.util.ConfigManager;
import pathplanning.util.FieldMap;
import pathplanning.util.NavGridVisualizer;
import pathplanning.util.Path;

import java.util.List;

/**
 * Main entry point for FRC Path Planning Service
 * Runs on Jetson Orin Nano Super and provides real-time path planning
 */
public class Main {
    
    // Service components
    private static AStar pathfinder;
    private static DynamicObstacleManager obstacleManager;
    private static RealTimeReplanner replanner;
    private static VisionDataReceiver visionReceiver;
    private static NetworkTablesBridge bridge;
    private static TrajectoryPublisher publisher;
    private static PathOptimizer pathOptimizer;
    private static VelocityProfiler profiler;
    private static FieldMap fieldMap;
    private static ConfigManager config;
    
    // State tracking
    private static Path currentPath = null;
    private static boolean running = true;
    
    public static void main(String[] args) {
        printBanner();
        
        // Parse command line arguments
        String mode = "teleop"; // Default mode
        String configPath = "config/planner_params.yaml";
        
        if (args.length > 0) {
            mode = args[0].toLowerCase();
        }
        if (args.length > 1) {
            configPath = args[1];
        }
        
        System.out.println("Starting path planning service in " + mode.toUpperCase() + " mode");
        System.out.println("Config: " + configPath + "\n");
        
        // Initialize service
        try {
            initialize(configPath);
        } catch (Exception e) {
            System.err.println("ERROR: Failed to initialize path planning service!");
            e.printStackTrace();
            System.exit(1);
        }
        
        // Add shutdown hook
        Runtime.getRuntime().addShutdownHook(new Thread(() -> {
            System.out.println("\nShutting down path planning service...");
            running = false;
            cleanup();
        }));
        
        // Run appropriate mode
        switch (mode) {
            case "auto":
            case "autonomous":
                runAutoMode();
                break;
            case "teleop":
            case "teleoperated":
                runTeleopMode();
                break;
            case "test":
                runTestMode();
                break;
            default:
                System.err.println("Unknown mode: " + mode);
                System.err.println("Valid modes: auto, teleop, test");
                System.exit(1);
        }
    }
    
    /**
     * Initialize all path planning components
     */
    private static void initialize(String configPath) {
        System.out.println("Initializing path planning service...\n");
        
        // Load configuration
        System.out.print("Loading configuration... ");
        config = new ConfigManager(configPath);
        System.out.println("✓");
        
        // Load field map
        System.out.print("Loading field map... ");
        String fieldConfigPath = config.getString("field.config_path", "config/navgrid.json");
        fieldMap = new FieldMap(fieldConfigPath);
        System.out.println("✓");
        
        // Show field statistics
        if (fieldMap.hasNavGrid()) {
            System.out.println("\nNavigation Grid:");
            NavGridVisualizer.printStats(fieldMap.getNavGridData());
            System.out.println();
        }
        
        // Initialize A* pathfinder
        System.out.print("Initializing A* pathfinder... ");
        pathfinder = new AStar(
            fieldMap.getStaticObstacles(),
            config.getDouble("planning.heuristic_weight", 1.0)
        );
        System.out.println("✓");
        
        // Initialize dynamic obstacle manager
        System.out.print("Initializing obstacle manager... ");
        obstacleManager = new DynamicObstacleManager(
            config.getDouble("robot.radius", 0.4),
            config.getDouble("obstacles.prediction_time", 1.0),
            config.getInt("obstacles.expiry_time_ms", 2000)
        );
        System.out.println("✓");
        
        // Initialize replanner
        System.out.print("Initializing replanner... ");
        replanner = new RealTimeReplanner(
            config.getInt("planning.frequency_hz", 20),
            config.getDouble("planning.deviation_threshold", 0.3),
            0.2
        );
        System.out.println("✓");
        
        // Initialize vision receiver
        System.out.print("Initializing vision interface... ");
        String visionTable = config.getString("network.vision_table", "vision");
        visionReceiver = new VisionDataReceiver(visionTable);
        System.out.println("✓");
        if (visionReceiver.isVisionActive()) {
            System.out.println("  ✓ Vision system active");
            System.out.println("    Cameras: " + visionReceiver.getActiveCameraCount());
            System.out.println("    Latency: " + String.format("%.1f ms", visionReceiver.getLatency()));
        } else {
            System.out.println("  ⚠️ Vision system offline");
        }
        
        // Initialize network communication
        System.out.print("Initializing NetworkTables... ");
        String plannerTable = config.getString("network.planner_table", "pathplanner");
        bridge = new NetworkTablesBridge(plannerTable);
        publisher = new TrajectoryPublisher(plannerTable);
        System.out.println("✓");
        
        // Initialize optimizers
        System.out.print("Initializing path optimizer... ");
        pathOptimizer = new PathOptimizer(
            config.getDouble("optimization.max_curvature", 2.0),
            config.getDouble("robot.max_velocity", 3.0),
            config.getDouble("robot.max_acceleration", 2.0)
        );
        System.out.println("✓");
        
        System.out.print("Initializing velocity profiler... ");
        profiler = new VelocityProfiler(
            config.getDouble("robot.max_velocity", 3.0),
            config.getDouble("robot.max_acceleration", 2.0),
            config.getDouble("robot.max_angular_velocity", Math.PI)
        );
        System.out.println("✓");
        
        System.out.println("\n=== Initialization Complete ===\n");
    }
    
    /**
     * Run autonomous mode
     */
    private static void runAutoMode() {
        System.out.println("=== AUTONOMOUS MODE ===");
        System.out.println("Waiting for target poses from auto sequence...\n");
        
        int segmentCount = 0;
        
        while (running) {
            try {
                Pose2d targetPose = bridge.getTargetPose();
                
                if (targetPose != null) {
                    segmentCount++;
                    Pose2d currentPose = bridge.getRobotPose();
                    
                    System.out.println("\n--- Auto Segment " + segmentCount + " ---");
                    System.out.println("Start: " + formatPose(currentPose));
                    System.out.println("Goal:  " + formatPose(targetPose));
                    
                    Path path = planPath(currentPose, targetPose);
                    
                    if (path != null) {
                        publishPath(path, currentPose);
                        
                        // Wait for robot to reach target
                        bridge.clearTarget();
                    } else {
                        System.err.println("Failed to plan segment " + segmentCount);
                        bridge.setPathStatus("failed");
                    }
                }
                
                Thread.sleep(100);
                
            } catch (InterruptedException e) {
                break;
            } catch (Exception e) {
                System.err.println("Error in auto mode: " + e.getMessage());
                e.printStackTrace();
            }
        }
    }
    
    /**
     * Run teleoperated mode with real-time replanning
     */
    private static void runTeleopMode() {
        System.out.println("=== TELEOPERATED MODE ===");
        System.out.println("Real-time path planning active");
        System.out.println("Waiting for driver commands...\n");
        
        int loopCount = 0;
        int pathCount = 0;
        long lastStatusTime = System.currentTimeMillis();
        final boolean visionEnabled = config.getBoolean("vision.enabled", true);
        
        while (running) {
            loopCount++;
            
            try {
                // Get current state
                Pose2d currentPose = bridge.getRobotPose();
                Pose2d targetPose = bridge.getTargetPose();
                List<VisionDataReceiver.DetectedObject> detections = visionEnabled
                    ? visionReceiver.getLatestObjects()
                    : List.of();
                
                if (visionEnabled) {
                    obstacleManager.updateDynamicObstacles(detections);
                }
                
                // Periodic status update
                if (System.currentTimeMillis() - lastStatusTime > 5000) {
                    System.out.println("Status: Running (" + pathCount + " paths planned)");
                    if (visionEnabled) {
                        boolean active = visionReceiver.isVisionActive();
                        System.out.println("📹 Vision: " + (active ? "active" : "offline"));
                        if (active) {
                            System.out.println("  Cameras: " + visionReceiver.getActiveCameraCount());
                            System.out.println("  Latency: " + String.format("%.1f ms", visionReceiver.getLatency()));
                        }
                        System.out.println("  Detections: " + detections.size());
                        for (int i = 0; i < Math.min(3, detections.size()); i++) {
                            System.out.println("    " + detections.get(i));
                        }
                        if (detections.size() > 3) {
                            System.out.println("    ... (" + (detections.size() - 3) + " more)");
                        }
                    } else {
                        System.out.println("📹 Vision: disabled via config");
                    }
                    lastStatusTime = System.currentTimeMillis();
                }
                
                if (targetPose != null) {
                    // Check if replanning is needed
                    boolean shouldReplan = replanner.shouldReplan(
                        currentPose,
                        currentPath,
                        obstacleManager
                    );
                    
                    if (shouldReplan) {
                        pathCount++;
                        
                        if (loopCount % 20 == 0) { // Log every second
                            System.out.println("\nReplanning #" + pathCount);
                            System.out.println("  Position: " + formatPose(currentPose));
                            System.out.println("  Target: " + formatPose(targetPose));
                            System.out.println("  Obstacles: " + obstacleManager.getObstacleCount());
                        }
                        
                        Path path = planPath(currentPose, targetPose);
                        
                        if (path != null) {
                            publishPath(path, currentPose);
                            currentPath = path;
                        } else {
                            System.err.println("  No path found!");
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
                
                // Maintain loop rate
                int sleepMs = 1000 / config.getInt("planning.frequency_hz", 20);
                Thread.sleep(sleepMs);
                
            } catch (InterruptedException e) {
                break;
            } catch (Exception e) {
                System.err.println("Error in teleop loop: " + e.getMessage());
                e.printStackTrace();
            }
        }
    }
    
    /**
     * Run test mode - single path planning example
     */
    private static void runTestMode() {
        System.out.println("=== TEST MODE ===\n");
        
        // Example test path
        Pose2d start = new Pose2d(1.5, 4.0, new Rotation2d(0));
        Pose2d goal = new Pose2d(15.5, 4.0, new Rotation2d(Math.PI));
        
        System.out.println("Planning test path:");
        System.out.println("  Start: " + formatPose(start));
        System.out.println("  Goal:  " + formatPose(goal));
        System.out.println();
        
        Path path = planPath(start, goal);
        
        if (path != null) {
            System.out.println("\n✓ Test path planned successfully!");
            publishPath(path, start);
            
            // Print first few waypoints
            System.out.println("\nPath waypoints:");
            for (int i = 0; i < Math.min(5, path.size()); i++) {
                System.out.println("  [" + i + "] " + formatPose(path.get(i)));
            }
            if (path.size() > 5) {
                System.out.println("  ... (" + (path.size() - 5) + " more)");
            }
        } else {
            System.err.println("\n✗ Test path planning failed!");
        }
        
        System.out.println("\n=== Test Complete ===");
    }
    
    /**
     * Plan a path from start to goal
     */
    private static Path planPath(Pose2d start, Pose2d goal) {
        long startTime = System.currentTimeMillis();
        
        // Find path
        Path path = pathfinder.findPath(
            start,
            goal,
            obstacleManager.getDynamicObstacles()
        );
        
        if (path == null) {
            return null;
        }
        
        // Optimize if enabled
        if (config.getBoolean("optimization.smoothing_enabled", true)) {
            path = PathSmoother.simplify(path, fieldMap.getStaticObstacles());
            path = PathSmoother.bezierSmooth(
                path,
                config.getInt("optimization.bezier_points_per_segment", 10)
            );
            path = pathOptimizer.constrainToDynamics(path);
        }
        
        long planningTime = System.currentTimeMillis() - startTime;
        bridge.publishPlanningTime(planningTime);
        
        if (config.getBoolean("logging.log_planning_time", true)) {
            System.out.println("  Planning time: " + planningTime + " ms");
            System.out.println("  Path length: " + String.format("%.2f m", path.getTotalLength()));
            System.out.println("  Waypoints: " + path.size());
        }
        
        return path;
    }
    
    /**
     * Publish path as trajectory
     */
    private static void publishPath(Path path, Pose2d currentPose) {
        // Generate velocity profile
        VelocityProfiler.Trajectory trajectory = profiler.generateProfile(path, currentPose);
        
        // Publish
        bridge.publishTrajectory(trajectory);
        publisher.publishForVisualization(trajectory);
        bridge.setPathStatus("following");
        
        if (config.getBoolean("logging.enabled", true)) {
            System.out.println("  Trajectory duration: " + 
                String.format("%.2f s", trajectory.getTotalTime()));
        }
    }
    
    /**
     * Cleanup resources
     */
    private static void cleanup() {
        if (publisher != null) {
            publisher.clear();
        }
        if (bridge != null) {
            bridge.setPathStatus("stopped");
        }
        System.out.println("Cleanup complete");
    }
    
    /**
     * Format pose for display
     */
    private static String formatPose(Pose2d pose) {
        return String.format("(%.2f, %.2f, %.1f°)",
            pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }
    
    /**
     * Print startup banner
     */
    private static void printBanner() {
        System.out.println();
        System.out.println("╔════════════════════════════════════════════════════════╗");
        System.out.println("║                                                        ║");
        System.out.println("║        FRC Path Planning Service - Team 1086           ║");
        System.out.println("║              On-The-Fly A* Path Planner                ║");
        System.out.println("║                                                        ║");
        System.out.println("║  Real-time pathfinding for FRC robots                  ║");
        System.out.println("║  Running on Jetson Orin Nano Super                     ║");
        System.out.println("║                                                        ║");
        System.out.println("║  Developer: @Antigro09                                 ║");
        System.out.println("║  Repository: 1086-On-The-Fly-A-Star                    ║");
        System.out.println("║                                                        ║");
        System.out.println("╚════════════════════════════════════════════════════════╝");
        System.out.println();
    }
}