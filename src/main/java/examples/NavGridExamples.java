package pathplanning.examples;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import pathplanning.core.AStar;
import pathplanning.core.PathSmoother;
import pathplanning.optimization.PathOptimizer;
import pathplanning.optimization.VelocityProfiler;
import pathplanning.util.FieldMap;
import pathplanning.util.NavGridLoader;
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
        NavGridLoader.NavGridData navGrid = fieldMap.getNavGridData();

        Pose2d start = pickTraversablePose(navGrid, 0.2, Rotation2d.fromDegrees(0));
        Pose2d goal = pickTraversablePose(navGrid, 0.8, Rotation2d.fromDegrees(180));

        if (start == null || goal == null) {
            System.err.println("ERROR: Unable to locate traversable start/goal in navgrid");
            return;
        }
        
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

    private static Pose2d pickTraversablePose(NavGridLoader.NavGridData navGrid,
                                             double columnFraction,
                                             Rotation2d heading) {
        if (navGrid == null) {
            return null;
        }

        int rows = navGrid.getGridRows();
        int cols = navGrid.getGridCols();
        int targetCol = Math.min(cols - 1, Math.max(0, (int) Math.round(cols * columnFraction)));
        int centerRow = rows / 2;

        for (int offset = 0; offset < rows; offset++) {
            int rowUp = centerRow - offset;
            if (rowUp >= 0) {
                Pose2d pose = poseIfTraversable(navGrid, rowUp, targetCol, heading);
                if (pose != null) {
                    return pose;
                }
            }

            int rowDown = centerRow + offset;
            if (rowDown < rows) {
                Pose2d pose = poseIfTraversable(navGrid, rowDown, targetCol, heading);
                if (pose != null) {
                    return pose;
                }
            }
        }

        return null;
    }

    private static Pose2d poseIfTraversable(NavGridLoader.NavGridData navGrid,
                                            int row,
                                            int col,
                                            Rotation2d heading) {
        boolean[][] grid = navGrid.grid;
        if (row < 0 || row >= grid.length || col < 0 || col >= grid[0].length) {
            return null;
        }

        if (!grid[row][col]) {
            return null;
        }

        double nodeSize = navGrid.nodeSize;
        double x = (col + 0.5) * nodeSize;
        double y = (grid.length - row - 0.5) * nodeSize;
        return new Pose2d(x, y, heading);
    }
}