package pathplanning.util;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import pathplanning.core.VisibilityGraph;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * Manages field geometry and static obstacles
 */
public class FieldMap {
    private final double fieldWidth;
    private final double fieldLength;
    private final List<VisibilityGraph.Obstacle> staticObstacles;
    private static final double DEFAULT_FIELD_WIDTH = 8.21;
    private static final double DEFAULT_FIELD_LENGTH = 16.54;
    
    public FieldMap(String configPath) {
        this.staticObstacles = new ArrayList<>();
        double width = DEFAULT_FIELD_WIDTH;
        double length = DEFAULT_FIELD_LENGTH;
        boolean configLoaded = false;

        // Load from config file
        try {
            FieldDimensions dimensions = loadFromConfig(configPath);
            width = dimensions.width();
            length = dimensions.length();
            configLoaded = true;
        } catch (IOException e) {
            System.err.println("Failed to load field config: " + e.getMessage());
        }

        this.fieldWidth = width;
        this.fieldLength = length;

        if (!configLoaded || staticObstacles.isEmpty()) {
            staticObstacles.clear();
            loadDefaultObstacles();
        }
    }
    
    /**
     * Load field configuration from JSON
     */
    private FieldDimensions loadFromConfig(String configPath) throws IOException {
        ObjectMapper mapper = new ObjectMapper();
        JsonNode root = mapper.readTree(new File(configPath));
        JsonNode fieldNode = root.get("field");
        double width = fieldNode != null
            ? fieldNode.path("width_meters").asDouble(DEFAULT_FIELD_WIDTH)
            : DEFAULT_FIELD_WIDTH;
        double length = fieldNode != null
            ? fieldNode.path("length_meters").asDouble(DEFAULT_FIELD_LENGTH)
            : DEFAULT_FIELD_LENGTH;
        
        JsonNode obstaclesNode = root.get("obstacles");
        if (obstaclesNode != null && obstaclesNode.isArray()) {
            for (JsonNode obstacleNode : obstaclesNode) {
                JsonNode vertices = obstacleNode.get("vertices");
                
                List<Double> xPoints = new ArrayList<>();
                List<Double> yPoints = new ArrayList<>();
                
                for (JsonNode vertex : vertices) {
                    xPoints.add(vertex.get("x").asDouble());
                    yPoints.add(vertex.get("y").asDouble());
                }
                
                double[] xArray = xPoints.stream().mapToDouble(Double::doubleValue).toArray();
                double[] yArray = yPoints.stream().mapToDouble(Double::doubleValue).toArray();
                
                staticObstacles.add(new VisibilityGraph.Obstacle(xArray, yArray));
            }
        }

        return new FieldDimensions(width, length);
    }
    
    /**
     * Load default obstacles (field boundaries and common structures)
     */
    private void loadDefaultObstacles() {
        // Field boundaries
        addFieldBoundaries();
        
        // Example: Charging station (adjust for your game)
        double[] chargingX = {5.0, 7.0, 7.0, 5.0};
        double[] chargingY = {2.0, 2.0, 4.0, 4.0};
        staticObstacles.add(new VisibilityGraph.Obstacle(chargingX, chargingY));
    }
    
    /**
     * Add field boundaries as obstacles
     */
    private void addFieldBoundaries() {
        double margin = 0.5; // Margin from field edge
        
        // Bottom boundary
        double[] bottomX = {-margin, fieldLength + margin, fieldLength + margin, -margin};
        double[] bottomY = {-margin, -margin, 0, 0};
        staticObstacles.add(new VisibilityGraph.Obstacle(bottomX, bottomY));
        
        // Top boundary
        double[] topX = {-margin, fieldLength + margin, fieldLength + margin, -margin};
        double[] topY = {fieldWidth, fieldWidth, fieldWidth + margin, fieldWidth + margin};
        staticObstacles.add(new VisibilityGraph.Obstacle(topX, topY));
        
        // Left boundary
        double[] leftX = {-margin, 0, 0, -margin};
        double[] leftY = {-margin, -margin, fieldWidth + margin, fieldWidth + margin};
        staticObstacles.add(new VisibilityGraph.Obstacle(leftX, leftY));
        
        // Right boundary
        double[] rightX = {fieldLength, fieldLength + margin, fieldLength + margin, fieldLength};
        double[] rightY = {-margin, -margin, fieldWidth + margin, fieldWidth + margin};
        staticObstacles.add(new VisibilityGraph.Obstacle(rightX, rightY));
    }
    
    public List<VisibilityGraph.Obstacle> getStaticObstacles() {
        return new ArrayList<>(staticObstacles);
    }
    
    public double getFieldWidth() {
        return fieldWidth;
    }
    
    public double getFieldLength() {
        return fieldLength;
    }
    
    /**
     * Check if pose is within field boundaries
     */
    public boolean isWithinField(double x, double y) {
        return x >= 0 && x <= fieldLength && y >= 0 && y <= fieldWidth;
    }

    private record FieldDimensions(double width, double length) {}
}