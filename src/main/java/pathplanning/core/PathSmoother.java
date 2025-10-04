package pathplanning.core;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import pathplanning.util.Path;

import java.util.ArrayList;
import java.util.List;

/**
 * Smooths paths using various algorithms
 */
public class PathSmoother {
    
    /**
     * Simplify path by removing unnecessary waypoints
     */
    public static Path simplify(Path path, List<VisibilityGraph.Obstacle> obstacles) {
        if (path.size() <= 2) {
            return path;
        }
        
        List<Pose2d> simplified = new ArrayList<>();
        simplified.add(path.get(0));
        
        int current = 0;
        while (current < path.size() - 1) {
            int farthest = current + 1;
            
            // Find farthest visible waypoint
            for (int i = current + 2; i < path.size(); i++) {
                if (isVisible(path.get(current), path.get(i), obstacles)) {
                    farthest = i;
                }
            }
            
            simplified.add(path.get(farthest));
            current = farthest;
        }
        
        return new Path(simplified);
    }
    
    /**
     * Smooth path using Bezier curves
     */
    public static Path bezierSmooth(Path path, int pointsPerSegment) {
        if (path.size() <= 2) {
            return path;
        }
        
        List<Pose2d> smoothed = new ArrayList<>();
        smoothed.add(path.get(0));
        
        for (int i = 0; i < path.size() - 1; i++) {
            Pose2d p0 = path.get(i);
            Pose2d p3 = path.get(i + 1);
            
            // Calculate control points
            double distance = p0.getTranslation().getDistance(p3.getTranslation());
            double controlDist = distance * 0.33;
            
            Translation2d p1 = p0.getTranslation().plus(
                new Translation2d(controlDist, p0.getRotation())
            );
            Translation2d p2 = p3.getTranslation().minus(
                new Translation2d(controlDist, p3.getRotation())
            );
            
            // Generate points along cubic Bezier curve
            for (int j = 1; j <= pointsPerSegment; j++) {
                double t = (double) j / pointsPerSegment;
                Translation2d point = cubicBezier(
                    p0.getTranslation(), p1, p2, p3.getTranslation(), t
                );
                
                // Interpolate rotation
                Rotation2d rotation = p0.getRotation().interpolate(
                    p3.getRotation(), t
                );
                
                if (j < pointsPerSegment || i == path.size() - 2) {
                    smoothed.add(new Pose2d(point, rotation));
                }
            }
        }
        
        return new Path(smoothed);
    }
    
    /**
     * Cubic Bezier curve calculation
     */
    private static Translation2d cubicBezier(Translation2d p0, Translation2d p1,
                                             Translation2d p2, Translation2d p3,
                                             double t) {
        double u = 1 - t;
        double tt = t * t;
        double uu = u * u;
        double uuu = uu * u;
        double ttt = tt * t;
        
        double x = uuu * p0.getX() +
                   3 * uu * t * p1.getX() +
                   3 * u * tt * p2.getX() +
                   ttt * p3.getX();
        
        double y = uuu * p0.getY() +
                   3 * uu * t * p1.getY() +
                   3 * u * tt * p2.getY() +
                   ttt * p3.getY();
        
        return new Translation2d(x, y);
    }
    
    /**
     * Check if path segment is visible (no obstacles)
     */
    private static boolean isVisible(Pose2d p1, Pose2d p2, 
                                     List<VisibilityGraph.Obstacle> obstacles) {
        double x1 = p1.getX();
        double y1 = p1.getY();
        double x2 = p2.getX();
        double y2 = p2.getY();
        
        for (VisibilityGraph.Obstacle obstacle : obstacles) {
            if (obstacle.intersectsLine(x1, y1, x2, y2)) {
                return false;
            }
        }
        
        return true;
    }
}