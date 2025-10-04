package pathplanning.optimization;

import edu.wpi.first.math.geometry.Pose2d;
import pathplanning.core.Node;

/**
 * Manages different cost functions for path optimization
 */
public class CostFunctionManager {
    private double distanceWeight = 1.0;
    private double curvatureWeight = 0.3;
    private double obstacleProximityWeight = 0.5;
    private double timeWeight = 0.8;
    
    /**
     * Calculate total cost between two nodes
     */
    public double calculateCost(Node from, Node to) {
        double distance = from.distanceTo(to);
        double curvatureCost = calculateCurvatureCost(from, to);
        
        return distanceWeight * distance + 
               curvatureWeight * curvatureCost;
    }
    
    /**
     * Calculate curvature cost (penalizes sharp turns)
     */
    private double calculateCurvatureCost(Node from, Node to) {
        double dx = to.getX() - from.getX();
        double dy = to.getY() - from.getY();
        double angle = Math.atan2(dy, dx);
        
        double rotationDiff = Math.abs(
            from.getRotation().getRadians() - angle
        );
        
        // Normalize to [-pi, pi]
        while (rotationDiff > Math.PI) {
            rotationDiff -= 2 * Math.PI;
        }
        while (rotationDiff < -Math.PI) {
            rotationDiff += 2 * Math.PI;
        }
        
        return Math.abs(rotationDiff);
    }
    
    /**
     * Calculate obstacle proximity cost
     */
    public double calculateObstacleProximityCost(Pose2d pose, 
                                                 double nearestObstacleDistance) {
        if (nearestObstacleDistance < 0.3) {
            return Double.POSITIVE_INFINITY; // Too close
        }
        
        if (nearestObstacleDistance > 1.0) {
            return 0; // Far enough
        }
        
        // Exponential cost as we get closer
        return obstacleProximityWeight * Math.exp(-nearestObstacleDistance);
    }
    
    /**
     * Calculate time-based cost (for dynamic planning)
     */
    public double calculateTimeCost(double travelTime, double targetTime) {
        double timeDiff = Math.abs(travelTime - targetTime);
        return timeWeight * timeDiff;
    }
    
    // Setters for weight tuning
    public void setDistanceWeight(double weight) {
        this.distanceWeight = weight;
    }
    
    public void setCurvatureWeight(double weight) {
        this.curvatureWeight = weight;
    }
    
    public void setObstacleProximityWeight(double weight) {
        this.obstacleProximityWeight = weight;
    }
    
    public void setTimeWeight(double weight) {
        this.timeWeight = weight;
    }
}