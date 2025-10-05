package pathplanning.examples;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;
import pathplanning.core.AStar;
import pathplanning.core.PathSmoother;
import pathplanning.optimization.PathOptimizer;
import pathplanning.optimization.VelocityProfiler;
import pathplanning.util.FieldMap;
import pathplanning.util.NavGridLoader;
import pathplanning.util.Path;

import static org.junit.jupiter.api.Assertions.*;

class NavGridExamplesTest {

    @Test
    void navGridPipelineGeneratesFeasibleTrajectory() {
        FieldMap fieldMap = new FieldMap("config/navgrid.json");
        assertTrue(fieldMap.hasNavGrid(), "navgrid.json should load nav grid data");
        NavGridLoader.NavGridData navGrid = fieldMap.getNavGridData();
        assertNotNull(navGrid, "navgrid data should be available");

        Pose2d start = pickTraversablePose(navGrid, 0.25, Rotation2d.fromDegrees(0));
        Pose2d goal = pickTraversablePose(navGrid, 0.75, Rotation2d.fromDegrees(180));

        assertNotNull(start, "start pose should be found in traversable space");
        assertNotNull(goal, "goal pose should be found in traversable space");

        assertTrue(fieldMap.isTraversable(start.getX(), start.getY()), "start position should be traversable");
        assertTrue(fieldMap.isTraversable(goal.getX(), goal.getY()), "goal position should be traversable");

        AStar pathfinder = new AStar(fieldMap.getStaticObstacles(), 1.0);
        Path rawPath = pathfinder.findPath(start, goal, fieldMap.getStaticObstacles());
        assertNotNull(rawPath, "A* should produce a path between start and goal");
        assertTrue(rawPath.size() > 1, "Path should contain multiple waypoints");

        Path simplified = PathSmoother.simplify(rawPath, fieldMap.getStaticObstacles());
        Path smoothed = PathSmoother.bezierSmooth(simplified, 10);

        PathOptimizer optimizer = new PathOptimizer(2.0, 3.0, 2.0);
        Path optimized = optimizer.constrainToDynamics(smoothed);
        assertNotNull(optimized);
        assertTrue(optimized.size() > 1, "Optimized path should retain waypoints");
        assertTrue(optimized.getTotalLength() > 0, "Optimized path length should be positive");

        VelocityProfiler profiler = new VelocityProfiler(3.0, 2.0, Math.PI);
        VelocityProfiler.Trajectory trajectory = profiler.generateProfile(optimized, start);

        assertNotNull(trajectory, "Velocity profiler should produce a trajectory");
        assertTrue(trajectory.getTotalTime() > 0, "Trajectory duration should be positive");
        assertFalse(trajectory.getStates().isEmpty(), "Trajectory should contain states");
    }

    private static Pose2d pickTraversablePose(NavGridLoader.NavGridData navGrid,
                                             double columnFraction,
                                             Rotation2d heading) {
        int rows = navGrid.getGridRows();
        int cols = navGrid.getGridCols();
        int targetCol = Math.min(cols - 1, Math.max(0, (int) (cols * columnFraction)));
        int centerRow = rows / 2;

        for (int offset = 0; offset < rows; offset++) {
            int rowUp = centerRow - offset;
            int rowDown = centerRow + offset;

            if (rowUp >= 0) {
                Pose2d pose = poseIfTraversable(navGrid, rowUp, targetCol, heading);
                if (pose != null) {
                    return pose;
                }
            }

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
        if (col < 0 || col >= grid[0].length) {
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
