package pathplanning.util;

import edu.wpi.first.math.geometry.Pose2d;

import java.util.ArrayList;
import java.util.List;

/**
 * Represents a path as a sequence of poses
 */
public class Path {
    private final List<Pose2d> waypoints;
    
    public Path() {
        this.waypoints = new ArrayList<>();
    }
    
    public Path(List<Pose2d> waypoints) {
        this.waypoints = new ArrayList<>(waypoints);
    }
    
    public void addWaypoint(Pose2d pose) {
        waypoints.add(pose);
    }
    
    public Pose2d get(int index) {
        return waypoints.get(index);
    }
    
    public int size() {
        return waypoints.size();
    }
    
    public boolean isEmpty() {
        return waypoints.isEmpty();
    }
    
    public List<Pose2d> getWaypoints() {
        return new ArrayList<>(waypoints);
    }
    
    /**
     * Calculate total path length
     */
    public double getTotalLength() {
        double length = 0;
        for (int i = 0; i < waypoints.size() - 1; i++) {
            length += waypoints.get(i).getTranslation()
                .getDistance(waypoints.get(i + 1).getTranslation());
        }
        return length;
    }
    
    /**
     * Get pose at specific distance along path
     */
    public Pose2d getPoseAtDistance(double distance) {
        if (waypoints.isEmpty()) {
            return null;
        }
        
        if (distance <= 0) {
            return waypoints.get(0);
        }
        
        double accumulated = 0;
        for (int i = 0; i < waypoints.size() - 1; i++) {
            Pose2d current = waypoints.get(i);
            Pose2d next = waypoints.get(i + 1);
            double segmentLength = current.getTranslation().getDistance(next.getTranslation());
            
            if (accumulated + segmentLength >= distance) {
                double t = (distance - accumulated) / segmentLength;
                return current.interpolate(next, t);
            }
            
            accumulated += segmentLength;
        }
        
        return waypoints.get(waypoints.size() - 1);
    }
    
    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder("Path with " + waypoints.size() + " waypoints:\n");
        for (int i = 0; i < waypoints.size(); i++) {
            Pose2d pose = waypoints.get(i);
            sb.append(String.format("  [%d] (%.2f, %.2f, %.2f°)\n", 
                i, pose.getX(), pose.getY(), pose.getRotation().getDegrees()));
        }
        sb.append(String.format("Total length: %.2f m", getTotalLength()));
        return sb.toString();
    }
}