package pathplanning.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Utility functions for geometry calculations
 */
public class GeometryUtils {
    
    /**
     * Check if point is inside polygon using ray casting algorithm
     */
    public static boolean pointInPolygon(double x, double y, double[] xPoints, double[] yPoints) {
        int nPoints = xPoints.length;
        boolean inside = false;
        
        for (int i = 0, j = nPoints - 1; i < nPoints; j = i++) {
            double xi = xPoints[i];
            double yi = yPoints[i];
            double xj = xPoints[j];
            double yj = yPoints[j];
            
            boolean intersect = ((yi > y) != (yj > y)) &&
                (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
            
            if (intersect) {
                inside = !inside;
            }
        }
        
        return inside;
    }
    
    /**
     * Calculate distance from point to line segment
     */
    public static double distanceToLineSegment(Translation2d point, 
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
    
    /**
     * Calculate angle between two poses
     */
    public static Rotation2d angleBetween(Pose2d from, Pose2d to) {
        double dx = to.getX() - from.getX();
        double dy = to.getY() - from.getY();
        return new Rotation2d(dx, dy);
    }
    
    /**
     * Check if two line segments intersect
     */
    public static boolean lineSegmentsIntersect(double x1, double y1, double x2, double y2,
                                                double x3, double y3, double x4, double y4) {
        double d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        
        if (Math.abs(d) < 1e-10) {
            return false; // Parallel
        }
        
        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / d;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / d;
        
        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }
    
    /**
     * Calculate curvature of path at given point
     */
    public static double calculateCurvature(Pose2d p1, Pose2d p2, Pose2d p3) {
        Translation2d t1 = p1.getTranslation();
        Translation2d t2 = p2.getTranslation();
        Translation2d t3 = p3.getTranslation();
        
        // Menger curvature formula
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
     * Clamp value between min and max
     */
    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
    
    /**
     * Normalize angle to [-pi, pi]
     */
    public static double normalizeAngle(double angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }
    
    /**
     * Calculate signed angle difference
     */
    public static double angleDifference(Rotation2d from, Rotation2d to) {
        return normalizeAngle(to.getRadians() - from.getRadians());
    }
    
    /**
     * Inflate polygon by given radius
     */
    public static double[][] inflatePolygon(double[] xPoints, double[] yPoints, double radius) {
        int n = xPoints.length;
        double[] newX = new double[n];
        double[] newY = new double[n];
        
        for (int i = 0; i < n; i++) {
            int prev = (i - 1 + n) % n;
            int next = (i + 1) % n;
            
            // Calculate edge normals
            double dx1 = xPoints[i] - xPoints[prev];
            double dy1 = yPoints[i] - yPoints[prev];
            double len1 = Math.sqrt(dx1 * dx1 + dy1 * dy1);
            double nx1 = -dy1 / len1;
            double ny1 = dx1 / len1;
            
            double dx2 = xPoints[next] - xPoints[i];
            double dy2 = yPoints[next] - yPoints[i];
            double len2 = Math.sqrt(dx2 * dx2 + dy2 * dy2);
            double nx2 = -dy2 / len2;
            double ny2 = dx2 / len2;
            
            // Average normal
            double nx = (nx1 + nx2) / 2;
            double ny = (ny1 + ny2) / 2;
            double len = Math.sqrt(nx * nx + ny * ny);
            
            if (len > 1e-6) {
                nx /= len;
                ny /= len;
            }
            
            // Offset point
            newX[i] = xPoints[i] + nx * radius;
            newY[i] = yPoints[i] + ny * radius;
        }
        
        return new double[][]{newX, newY};
    }
}