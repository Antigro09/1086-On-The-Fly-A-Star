package pathplanning.dynamic;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import pathplanning.core.VisibilityGraph;
import pathplanning.integration.VisionDataReceiver.DetectedObject;

import java.util.*;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Manages dynamic obstacles with motion tracking and prediction
 */
public class DynamicObstacleManager {
    private final Map<String, TrackedObstacle> trackedObjects;
    private final double robotRadius;
    private final double predictionTime;
    private final long expiryTimeMs;
    
    private static final double DEFAULT_PREDICTION_TIME = 1.0; // seconds
    private static final long DEFAULT_EXPIRY_MS = 2000; // milliseconds
    
    public DynamicObstacleManager(double robotRadius) {
        this(robotRadius, DEFAULT_PREDICTION_TIME, DEFAULT_EXPIRY_MS);
    }
    
    public DynamicObstacleManager(double robotRadius, double predictionTime, 
                                 long expiryTimeMs) {
        this.trackedObjects = new ConcurrentHashMap<>();
        this.robotRadius = robotRadius;
        this.predictionTime = predictionTime;
        this.expiryTimeMs = expiryTimeMs;
    }
    
    /**
     * Update obstacles from vision detections
     */
    public void updateDynamicObstacles(List<DetectedObject> visionObjects) {
        long currentTime = System.currentTimeMillis();

        for (DetectedObject obj : visionObjects) {
            if (!obj.isFresh(expiryTimeMs)) {
                continue;
            }

            String id = obj.getId();
            trackedObjects.compute(id, (key, tracked) -> {
                if (tracked == null) {
                    return new TrackedObstacle(obj, currentTime);
                }
                tracked.update(obj, currentTime);
                return tracked;
            });
        }

        // Remove expired obstacles, extend lifetime for tracked roboRIO objects
        trackedObjects.entrySet().removeIf(entry -> {
            TrackedObstacle tracked = entry.getValue();
            long timeout = tracked.isTracked() ? expiryTimeMs * 2 : expiryTimeMs;
            return (currentTime - tracked.getLastSeen()) > timeout;
        });
    }
    
    /**
     * Get all dynamic obstacles with predictions
     */
    public List<VisibilityGraph.Obstacle> getDynamicObstacles() {
        List<VisibilityGraph.Obstacle> obstacles = new ArrayList<>();
        
        for (TrackedObstacle tracked : trackedObjects.values()) {
            // Predict future position
            tracked.predictMotion(predictionTime);
            
            // Get inflated obstacle
            VisibilityGraph.Obstacle obstacle = tracked.getInflatedObstacle(robotRadius);
            if (obstacle != null) {
                obstacles.add(obstacle);
            }
        }
        
        return obstacles;
    }
    
    /**
     * Get number of tracked obstacles
     */
    public int getObstacleCount() {
        return trackedObjects.size();
    }
    
    /**
     * Clear all tracked obstacles
     */
    public void clear() {
        trackedObjects.clear();
    }
    
    /**
     * Inner class for tracking individual obstacles
     */
    private static class TrackedObstacle {
        private DetectedObject current;
        private final LinkedList<TimestampedPose> history;
        private Pose2d predictedPosition;
        private Translation2d velocity;
        private long lastSeen;
        
        private static final int MAX_HISTORY = 10;
        
        public TrackedObstacle(DetectedObject initial, long timestamp) {
            this.current = initial;
            this.history = new LinkedList<>();
            this.lastSeen = timestamp;
            this.velocity = new Translation2d();
            this.predictedPosition = initial.toPose2d();
        }
        
        public void update(DetectedObject newObj, long timestamp) {
            // Calculate velocity
            if (current != null) {
                double dt = (timestamp - lastSeen) / 1000.0;
                if (dt > 0.01) { // Avoid division by very small numbers
                    double dx = newObj.getX() - current.getX();
                    double dy = newObj.getY() - current.getY();
                    velocity = new Translation2d(dx / dt, dy / dt);
                }
            }
            
            // Add to history
            history.add(new TimestampedPose(current.toPose2d(), lastSeen));
            if (history.size() > MAX_HISTORY) {
                history.removeFirst();
            }
            
            current = newObj;
            lastSeen = timestamp;
        }
        
        public void predictMotion(double timeAhead) {
            if (velocity != null && velocity.getNorm() > 0.01) {
                // Linear prediction with velocity
                Translation2d predicted = current.toPose2d().getTranslation()
                    .plus(velocity.times(timeAhead));
                predictedPosition = new Pose2d(predicted, current.toPose2d().getRotation());
            } else {
                predictedPosition = current.toPose2d();
            }
        }
        
        public VisibilityGraph.Obstacle getInflatedObstacle(double radius) {
            if (predictedPosition == null) {
                return null;
            }
            
            // Create rectangular obstacle around predicted position
            double width = current.getWidth() + 2 * radius;
            double height = current.getHeight() + 2 * radius;
            
            double x = predictedPosition.getX();
            double y = predictedPosition.getY();
            
            double[] xPoints = {
                x - width / 2, x + width / 2, 
                x + width / 2, x - width / 2
            };
            double[] yPoints = {
                y - height / 2, y - height / 2, 
                y + height / 2, y + height / 2
            };
            
            return new VisibilityGraph.Obstacle(xPoints, yPoints);
        }
        
        public long getLastSeen() {
            return lastSeen;
        }

        public boolean isTracked() {
            return current != null && current.isTracked();
        }
        
        private static class TimestampedPose {
            final Pose2d pose;
            final long timestamp;
            
            TimestampedPose(Pose2d pose, long timestamp) {
                this.pose = pose;
                this.timestamp = timestamp;
            }
        }
    }
}