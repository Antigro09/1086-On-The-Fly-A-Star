package pathplanning.integration;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.ArrayList;
import java.util.List;

/**
 * Receives vision data from FRC-AI-Vision system
 */
public class VisionDataReceiver {
    private final NetworkTable visionTable;
    private final String topicName;
    
    public VisionDataReceiver() {
        this("vision");
    }
    
    public VisionDataReceiver(String tableName) {
        this(tableName, "detected_objects");
    }
    
    public VisionDataReceiver(String tableName, String topicName) {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        this.visionTable = inst.getTable(tableName);
        this.topicName = topicName;
    }
    
    /**
     * Subscribe to vision data topic
     */
    public void subscribe(String topic) {
        // NetworkTables automatically subscribes when accessing entries
        visionTable.getEntry(topic);
    }
    
    /**
     * Get latest detected objects from vision system
     */
    public List<DetectedObject> getLatestObjects() {
        List<DetectedObject> objects = new ArrayList<>();
        
        // Get number of detected objects
        int numObjects = (int) visionTable.getEntry("num_objects").getDouble(0);
        
        for (int i = 0; i < numObjects; i++) {
            String prefix = "object_" + i + "_";
            
            // Read object data
            String id = visionTable.getEntry(prefix + "id").getString("");
            String className = visionTable.getEntry(prefix + "class").getString("");
            double confidence = visionTable.getEntry(prefix + "confidence").getDouble(0);
            double x = visionTable.getEntry(prefix + "x").getDouble(0);
            double y = visionTable.getEntry(prefix + "y").getDouble(0);
            double width = visionTable.getEntry(prefix + "width").getDouble(0);
            double height = visionTable.getEntry(prefix + "height").getDouble(0);
            double rotation = visionTable.getEntry(prefix + "rotation").getDouble(0);
            
            if (!id.isEmpty()) {
                DetectedObject obj = new DetectedObject(
                    id, className, confidence, x, y, width, height, rotation
                );
                objects.add(obj);
            }
        }
        
        return objects;
    }
    
    /**
     * Get vision system status
     */
    public boolean isVisionActive() {
        return visionTable.getEntry("active").getBoolean(false);
    }
    
    /**
     * Get vision system latency
     */
    public double getLatency() {
        return visionTable.getEntry("latency_ms").getDouble(0);
    }
    
    /**
     * Represents a detected object from vision
     */
    public static class DetectedObject {
        private final String id;
        private final String className;
        private final double confidence;
        private final double x;
        private final double y;
        private final double width;
        private final double height;
        private final double rotation;
        
        public DetectedObject(String id, String className, double confidence,
                            double x, double y, double width, double height,
                            double rotation) {
            this.id = id;
            this.className = className;
            this.confidence = confidence;
            this.x = x;
            this.y = y;
            this.width = width;
            this.height = height;
            this.rotation = rotation;
        }
        
        public String getId() {
            return id;
        }
        
        public String getClassName() {
            return className;
        }
        
        public double getConfidence() {
            return confidence;
        }
        
        public double getX() {
            return x;
        }
        
        public double getY() {
            return y;
        }
        
        public double getWidth() {
            return width;
        }
        
        public double getHeight() {
            return height;
        }
        
        public double getRotation() {
            return rotation;
        }
        
        public Pose2d toPose2d() {
            return new Pose2d(x, y, Rotation2d.fromDegrees(rotation));
        }
        
        @Override
        public String toString() {
            return String.format("DetectedObject[id=%s, class=%s, conf=%.2f, pos=(%.2f, %.2f)]",
                id, className, confidence, x, y);
        }
    }
}