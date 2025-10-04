package pathplanning.optimization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import pathplanning.core.VisibilityGraph;
import pathplanning.util.Path;

import java.util.ArrayList;
import java.util.List;

/**
 * Optimizes paths for robot constraints and smoothness
 */
public class PathOptimizer {
    private final double maxCurvature;
    private final double maxVelocity;
    private final double maxAcceleration;
    
    public PathOptimizer(double maxCurvature, double maxVelocity, 
                        double maxAcceleration) {
        this.maxCurvature = maxCurvature;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
    }
    
    /**
     * Optimize path to respect robot dynamics
     */
    public Path constrainToDynamics(Path path) {
        if (path.size() <= 2) {
            return path;
        }
        
        List<Pose2d> optimized = new ArrayList<>();
        optimized.add(path.get(0));
        
        for (int i = 1; i < path.size() - 1; i++) {
            Pose2d prev = path.get(i - 1);
            Pose2d current = path.get(i);
            Pose2d next = path.get(i + 1);
            
            // Calculate curvature at this point
            double curvature = calculateCurvature(prev, current, next);
            
            if (Math.abs(curvature) > maxCurvature) {
                // Insert additional waypoints to reduce curvature
                List<Pose2d> interpolated = interpolateSegment(prev, next, 3);
                optimized.addAll(interpolated.subList(1, interpolated.size() - 1));
            } else {
                optimized.add(current);
            }
        }
        
        optimized.add(path.get(path.size() - 1));
        
        return new Path(optimized);
    }
    
    /**
     * Calculate curvature at a point
     */
    private double calculateCurvature(Pose2d p1, Pose2d p2, Pose2d p3) {
        Translation2d t1 = p1.getTranslation();
        Translation2d t2 = p2.getTranslation();
        Translation2d t3 = p3.getTranslation();
        
        // Using Menger curvature formula
        double area = Math.abs(
            (t2.getX() - t1.getX()) * (t3.getY() - t1.getY()) -
            (t2.getY() - t1.getY()) * (t3.getX() - t1.getX())
        ) / 2.0;
        
        double a = t1.getDistance(t2);
        double b = t2.getDistance(t3);
        double c = t3.getDistance(t1);
        
        if (a * b * c < 1e-6) {
            return 0;
        }
        
        return 4 * area / (a * b * c);
    }
    
    /**
     * Interpolate between two poses
     */
    private List<Pose2d> interpolateSegment(Pose2d start, Pose2d end, int numPoints) {
        List<Pose2d> interpolated = new ArrayList<>();
        interpolated.add(start);
        
        for (int i = 1; i < numPoints; i++) {
            double t = (double) i / numPoints;
            Pose2d interpolated_pose = start.interpolate(end, t);
            interpolated.add(interpolated_pose);
        }
        
        interpolated.add(end);
        return interpolated;
    }
    
    /**
     * Optimize path length while avoiding obstacles
     */
    public Path shortcutPath(Path path, List<VisibilityGraph.Obstacle> obstacles) {
        if (path.size() <= 2) {
            return path;
        }
        
        List<Pose2d> shortcut = new ArrayList<>();
        shortcut.add(path.get(0));
        
        int i = 0;
        while (i < path.size() - 1) {
            int farthest = i + 1;
            
            // Find farthest visible waypoint
            for (int j = i + 2; j < path.size(); j++) {
                if (isVisible(path.get(i), path.get(j), obstacles)) {
                    farthest = j;
                }
            }
            
            if (farthest > i + 1) {
                // Can shortcut
                shortcut.add(path.get(farthest));
            } else {
                // Can't shortcut, add next point
                shortcut.add(path.get(i + 1));
            }
            
            i = farthest;
        }
        
        return new Path(shortcut);
    }
    
    /**
     * Check visibility between two poses
     */
    private boolean isVisible(Pose2d p1, Pose2d p2, 
                              List<VisibilityGraph.Obstacle> obstacles) {
        for (VisibilityGraph.Obstacle obstacle : obstacles) {
            if (obstacle.intersectsLine(
                p1.getX(), p1.getY(),
                p2.getX(), p2.getY()
            )) {
                return false;
            }
        }
        return true;
    }
}