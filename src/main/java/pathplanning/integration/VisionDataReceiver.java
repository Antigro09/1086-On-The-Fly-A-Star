package pathplanning.integration;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.ArrayList;
import java.util.List;

/**
 * Receives vision data from FRC-AI-Vision system (mainFRCVision.py)
 * Supports both raw detections and field-relative poses from roboRIO
 */
public class VisionDataReceiver {
    private final NetworkTable visionTable;
    private final NetworkTable fusedTable;
    private final NetworkTable robotTable;
    private final String topicName;
    
    // Configuration
    private static final double MIN_CONFIDENCE = 0.45;
    private static final double ALGAE_RADIUS = 0.1905; // 7.5 inches = 0.1905m
    private static final long MAX_AGE_MS = 2000; // 2 seconds max age
    
    public VisionDataReceiver() {
        this("Algae Detection", "detected_objects");
    }

    public VisionDataReceiver(String tableName) {
        this(tableName, "detected_objects");
    }
    
    public VisionDataReceiver(String tableName, String topicName) {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        this.visionTable = inst.getTable(tableName);
        this.fusedTable = inst.getTable(tableName).getSubTable("Fused");
        this.robotTable = inst.getTable("robot"); // For field-relative poses
        this.topicName = topicName;
    }
    
    /**
     * Get field-relative algae detections from roboRIO
     * These are already converted to field coordinates with last-known-position tracking
     */
    public List<DetectedObject> getFieldRelativeObjects() {
        List<DetectedObject> objects = new ArrayList<>();
        
        // Read from robot table (field-relative poses from roboRIO)
        int numObjects = (int) robotTable.getEntry("algae/field_count").getDouble(0);
        
        for (int i = 0; i < numObjects; i++) {
            String prefix = "algae/field/" + i + "/";
            
            // Read field-relative pose
            double x = robotTable.getEntry(prefix + "x").getDouble(0);
            double y = robotTable.getEntry(prefix + "y").getDouble(0);
            double confidence = robotTable.getEntry(prefix + "confidence").getDouble(0);
            long timestamp = robotTable.getEntry(prefix + "timestamp").getInteger(0);
            boolean isTracked = robotTable.getEntry(prefix + "is_tracked").getBoolean(false);
            
            // Check if detection is recent enough
            long age = System.currentTimeMillis() - timestamp;
            if (age > MAX_AGE_MS) {
                continue; // Too old
            }
            
            // Check minimum confidence
            if (confidence < MIN_CONFIDENCE) {
                continue;
            }
            
            DetectedObject obj = new DetectedObject(
                "algae_" + i,
                "algae",
                confidence,
                x, y,
                ALGAE_RADIUS * 2, // width (diameter)
                ALGAE_RADIUS * 2, // height (diameter)
                0.0, // rotation
                isTracked,
                timestamp
            );
            objects.add(obj);
        }
        
        return objects;
    }
    
    /**
     * Get raw fused detections from mainFRCVision.py (robot-relative)
     * Use this if you want to do field-relative conversion yourself
     */
    public List<DetectedObject> getFusedObjects() {
        List<DetectedObject> objects = new ArrayList<>();
        
        // Read fused detections from mainFRCVision.py
        int numFused = (int) fusedTable.getEntry("algae/count").getDouble(0);
        
        for (int i = 0; i < numFused; i++) {
            String prefix = "algae/" + i + "/";
            
            // Read pose (robot-relative)
            double[] poseData = fusedTable.getEntry(prefix + "pose").getDoubleArray(new double[]{0, 0, 0});
            double x = poseData.length > 0 ? poseData[0] : 0;
            double y = poseData.length > 1 ? poseData[1] : 0;
            
            double confidence = fusedTable.getEntry(prefix + "avg_confidence").getDouble(0);
            String method = fusedTable.getEntry(prefix + "method").getString("");
            double quality = fusedTable.getEntry(prefix + "quality").getDouble(0);
            
            if (confidence < MIN_CONFIDENCE) {
                continue;
            }
            
            DetectedObject obj = new DetectedObject(
                "fused_" + i,
                "algae",
                confidence,
                x, y,
                ALGAE_RADIUS * 2,
                ALGAE_RADIUS * 2,
                0.0,
                false,
                System.currentTimeMillis()
            );
            obj.setMethod(method);
            obj.setQuality(quality);
            objects.add(obj);
        }
        
        return objects;
    }
    
    /**
     * Get latest detected objects (prioritizes field-relative)
     */
    public List<DetectedObject> getLatestObjects() {
        // Try field-relative first (from roboRIO)
        List<DetectedObject> fieldObjects = getFieldRelativeObjects();
        if (!fieldObjects.isEmpty()) {
            return fieldObjects;
        }
        
        // Fallback to fused robot-relative
        return getFusedObjects();
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
     * Get number of cameras providing data
     */
    public int getActiveCameraCount() {
        int count = 0;
        for (String key : visionTable.getKeys()) {
            if (key.startsWith("Cam_") && key.endsWith("/algae/count")) {
                count++;
            }
        }
        return count;
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
        private final boolean isTracked;
        private final long timestamp;
        
        private String method = "";
        private double quality = 0.0;
        
        public DetectedObject(String id, String className, double confidence,
                            double x, double y, double width, double height,
                            double rotation, boolean isTracked, long timestamp) {
            this.id = id;
            this.className = className;
            this.confidence = confidence;
            this.x = x;
            this.y = y;
            this.width = width;
            this.height = height;
            this.rotation = rotation;
            this.isTracked = isTracked;
            this.timestamp = timestamp;
        }
        
        // Getters
        public String getId() { return id; }
        public String getClassName() { return className; }
        public double getConfidence() { return confidence; }
        public double getX() { return x; }
        public double getY() { return y; }
        public double getWidth() { return width; }
        public double getHeight() { return height; }
        public double getRotation() { return rotation; }
        public boolean isTracked() { return isTracked; }
        public long getTimestamp() { return timestamp; }
        public String getMethod() { return method; }
        public double getQuality() { return quality; }
        
        public void setMethod(String method) { this.method = method; }
        public void setQuality(double quality) { this.quality = quality; }
        
        /**
         * Get age of detection in milliseconds
         */
        public long getAge() {
            return System.currentTimeMillis() - timestamp;
        }
        
        /**
         * Check if detection is fresh
         */
        public boolean isFresh(long maxAgeMs) {
            return getAge() <= maxAgeMs;
        }
        
        public Pose2d toPose2d() {
            return new Pose2d(x, y, Rotation2d.fromDegrees(rotation));
        }
        
        public Pose3d toPose3d() {
            return new Pose3d(x, y, 0.0, new Rotation3d());
        }
        
        /**
         * Get radius for circular obstacle (algae)
         */
        public double getRadius() {
            return Math.max(width, height) / 2.0;
        }
        
        @Override
        public String toString() {
            String trackedStr = isTracked ? "[TRACKED]" : "";
            String methodStr = method.isEmpty() ? "" : (" method=" + method);
            String qualityStr = quality > 0 ? String.format(" quality=%.2f", quality) : "";
            return String.format(
                "DetectedObject[id=%s, class=%s, conf=%.2f, pos=(%.2f, %.2f), age=%dms %s%s%s]",
                id, className, confidence, x, y, getAge(), trackedStr, methodStr, qualityStr
            );
        }
    }
}