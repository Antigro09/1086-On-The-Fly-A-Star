package pathplanning.examples;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import pathplanning.core.AStar;
import pathplanning.core.PathSmoother;
import pathplanning.optimization.PathOptimizer;
import pathplanning.optimization.VelocityProfiler;
import pathplanning.util.FieldMap;
import pathplanning.util.NavGridVisualizer;
import pathplanning.util.Path;

/**
 * Example using navgrid.json format for field configuration
 */
public class NavGridExamples {
    
    public static void main(String[] args) {
        System.out.println("=== NavGrid Path Planning Example ===\n");
        
        // Load field from navgrid.json
        FieldMap fieldMap = new FieldMap("config/navgrid.json");
        
        // Visualize the grid
        if (fieldMap.hasNavGrid()) {
            NavGridVisualizer.printStats(fieldMap.getNavGridData());
            // Uncomment to print full grid:
            // NavGridVisualizer.printGrid(fieldMap.getNavGridData());
        }
        
        // Initialize pathfinder
        AStar pathfinder = new AStar(fieldMap.getStaticObstacles(), 1.0);
        
        // Example: Plan path from blue alliance to red scoring
        Pose2d start = new Pose2d(1.5, 4.0, Rotation2d.fromDegrees(0));
        Pose2d goal = new Pose2d(15.5, 4.0, Rotation2d.fromDegrees(180));
        
        System.out.println("\nPlanning path:");
        System.out.println("  Start: " + formatPose(start));
        System.out.println("  Goal: " + formatPose(goal));
        
        // Check if start and goal are traversable
        if (!fieldMap.isTraversable(start.getX(), start.getY())) {
            System.err.println("ERROR: Start position is not traversable!");
            return;
        }
        
        if (!fieldMap.isTraversable(goal.getX(), goal.getY())) {
            System.err.println("ERROR: Goal position is not traversable!");
            return;
        }
        
        // Find path
        long startTime = System.currentTimeMillis();
        Path path = pathfinder.findPath(start, goal, fieldMap.getStaticObstacles());
        long planningTime = System.currentTimeMillis() - startTime;
        
        if (path == null) {
            System.err.println("ERROR: No path found!");
            return;
        }
        
        System.out.println("\n✓ Path found!");
        System.out.println("  Planning time: " + planningTime + " ms");
        System.out.println("  Raw waypoints: " + path.size());
        System.out.println("  Raw length: " + String.format("%.2f m", path.getTotalLength()));
        
        // Optimize path
        System.out.println("\nOptimizing path...");
        path = PathSmoother.simplify(path, fieldMap.getStaticObstacles());
        System.out.println("  After simplification: " + path.size() + " waypoints");
        
        path = PathSmoother.bezierSmooth(path, 10);
        System.out.println("  After smoothing: " + path.size() + " waypoints");
        
        PathOptimizer optimizer = new PathOptimizer(2.0, 3.0, 2.0);
        path = optimizer.constrainToDynamics(path);
        System.out.println("  Final length: " + String.format("%.2f m", path.getTotalLength()));
        
        // Generate velocity profile
        System.out.println("\nGenerating velocity profile...");
        VelocityProfiler profiler = new VelocityProfiler(3.0, 2.0, Math.PI);
        VelocityProfiler.Trajectory trajectory = profiler.generateProfile(path, start);
        
        System.out.println("  Duration: " + String.format("%.2f s", trajectory.getTotalTime()));
        System.out.println("  States: " + trajectory.getStates().size());
        
        // Print path waypoints
        System.out.println("\nPath Waypoints:");
        for (int i = 0; i < Math.min(10, path.size()); i++) {
            Pose2d pose = path.get(i);
            System.out.println("  [" + i + "] " + formatPose(pose));
        }
        if (path.size() > 10) {
            System.out.println("  ... (" + (path.size() - 10) + " more waypoints)");
        }
        
        System.out.println("\n=== Example Complete ===");
    }
    
    private static String formatPose(Pose2d pose) {
        return String.format("(%.2f, %.2f, %.1f°)",
            pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }
}