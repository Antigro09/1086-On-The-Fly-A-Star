package pathplanning.dynamic;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import pathplanning.core.VisibilityGraph;
import pathplanning.util.Path;

import java.util.List;

/**
 * Determines when replanning is necessary
 */
public class RealTimeReplanner {
    private final int frequencyHz;
    private long lastPlanTime;
    private double pathDeviationThreshold;
    private double improvementThreshold;
    
    private static final double DEFAULT_DEVIATION_THRESHOLD = 0.3; // meters
    private static final double DEFAULT_IMPROVEMENT_THRESHOLD = 0.2; // 20%
    
    public RealTimeReplanner(int frequencyHz) {
        this(frequencyHz, DEFAULT_DEVIATION_THRESHOLD, DEFAULT_IMPROVEMENT_THRESHOLD);
    }
    
    public RealTimeReplanner(int frequencyHz, double pathDeviationThreshold,
                            double improvementThreshold) {
        this.frequencyHz = frequencyHz;
        this.lastPlanTime = 0;
        this.pathDeviationThreshold = pathDeviationThreshold;
        this.improvementThreshold = improvementThreshold;
    }
    
    /**
     * Determine if replanning should occur
     */
    public boolean shouldReplan(Pose2d currentPose, Path currentPath,
                                DynamicObstacleManager obstacleManager) {
        long currentTime = System.currentTimeMillis();
        
        // Always replan if no current path
        if (currentPath == null || currentPath.size() == 0) {
            lastPlanTime = currentTime;
            return true;
        }
        
        // Check if enough time has passed
        long timeSinceLastPlan = currentTime - lastPlanTime;
        if (timeSinceLastPlan < 1000.0 / frequencyHz) {
            return false;
        }
        
        // Check path deviation
        double deviation = calculatePathDeviation(currentPose, currentPath);
        if (deviation > pathDeviationThreshold) {
            lastPlanTime = currentTime;
            return true;
        }
        
        // Check if path is blocked
        List<VisibilityGraph.Obstacle> obstacles = obstacleManager.getDynamicObstacles();
        if (isPathBlocked(currentPath, obstacles)) {
            lastPlanTime = currentTime;
            return true;
        }
        
        return false;
    }
    
    /**
     * Calculate deviation from current path
     */
    private double calculatePathDeviation(Pose2d currentPose, Path path) {
        double minDistance = Double.POSITIVE_INFINITY;
        
        for (int i = 0; i < path.size() - 1; i++) {
            Pose2d p1 = path.get(i);
            Pose2d p2 = path.get(i + 1);
            
            double distance = distanceToLineSegment(
                currentPose.getTranslation(),
                p1.getTranslation(),
                p2.getTranslation()
            );
            
            minDistance = Math.min(minDistance, distance);
        }
        
        return minDistance;
    }
    
    /**
     * Check if path is blocked by obstacles
     */
    private boolean isPathBlocked(Path path, List<VisibilityGraph.Obstacle> obstacles) {
        for (int i = 0; i < path.size() - 1; i++) {
            Pose2d start = path.get(i);
            Pose2d end = path.get(i + 1);
            
            for (VisibilityGraph.Obstacle obstacle : obstacles) {
                if (obstacle.intersectsLine(
                    start.getX(), start.getY(),
                    end.getX(), end.getY()
                )) {
                    return true;
                }
            }
        }
        return false;
    }
    
    /**
     * Calculate distance from point to line segment
     */
    private double distanceToLineSegment(Translation2d point,
                                        Translation2d lineStart,
                                        Translation2d lineEnd) {
        double dx = lineEnd.getX() - lineStart.getX();
        double dy = lineEnd.getY() - lineStart.getY();
        
        if (dx == 0 && dy == 0) {
            return point.getDistance(lineStart);
        }
        
        double t = ((point.getX() - lineStart.getX()) * dx +
                    (point.getY() - lineStart.getY()) * dy) / (dx * dx + dy * dy);
        
        t = Math.max(0, Math.min(1, t));
        
        Translation2d projection = new Translation2d(
            lineStart.getX() + t * dx,
            lineStart.getY() + t * dy
        );
        
        return point.getDistance(projection);
    }
    
    public void setPathDeviationThreshold(double threshold) {
        this.pathDeviationThreshold = threshold;
    }
    
    public void setImprovementThreshold(double threshold) {
        this.improvementThreshold = threshold;
    }
}