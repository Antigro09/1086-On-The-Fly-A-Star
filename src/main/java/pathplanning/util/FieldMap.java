package pathplanning.util;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import pathplanning.core.VisibilityGraph;
import pathplanning.util.NavGridLoader.NavGridData;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * Manages field geometry and static obstacles
 * Supports both field_config.json and navgrid.json formats
 */
public class FieldMap {
    private final double fieldWidth;
    private final double fieldLength;
    private final List<VisibilityGraph.Obstacle> staticObstacles;
    private NavGridData navGridData;
    
    /**
     * Constructor - automatically detects format
     */
    public FieldMap(String configPath) {
        this.staticObstacles = new ArrayList<>();
        
        // Try to detect format and load
        try {
            if (configPath.endsWith("navgrid.json")) {
                loadFromNavGrid(configPath);
                this.fieldWidth = navGridData.fieldHeight; // NavGrid uses x/y differently
                this.fieldLength = navGridData.fieldWidth;
            } else {
                loadFromConfig(configPath);
                this.fieldWidth = 8.21; // Default 2024 field
                this.fieldLength = 16.54;
            }
        } catch (IOException e) {
            System.err.println("Failed to load field config: " + e.getMessage());
            this.fieldWidth = 8.21;
            this.fieldLength = 16.54;
            loadDefaultObstacles();
        }
    }
    
    /**
     * Load from navgrid.json format (PathPlanner format)
     */
    private void loadFromNavGrid(String navGridPath) throws IOException {
        System.out.println("Loading field from navgrid.json format...");
        navGridData = NavGridLoader.loadFromFile(navGridPath);
        staticObstacles.addAll(navGridData.obstacles);
    }
    
    /**
     * Load field configuration from field_config.json
     */
    private void loadFromConfig(String configPath) throws IOException {
        System.out.println("Loading field from field_config.json format...");
        ObjectMapper mapper = new ObjectMapper();
        JsonNode root = mapper.readTree(new File(configPath));
        
        JsonNode obstaclesNode = root.get("obstacles");
        if (obstaclesNode != null && obstaclesNode.isArray()) {
            for (JsonNode obstacleNode : obstaclesNode) {
                String name = obstacleNode.get("name").asText();
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
                System.out.println("  Loaded obstacle: " + name);
            }
        }
    }
    
    /**
     * Load default obstacles (field boundaries and common structures)
     */
    private void loadDefaultObstacles() {
        System.out.println("Loading default field obstacles...");
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
    
    /**
     * Check if position is traversable (if using navgrid)
     */
    public boolean isTraversable(double x, double y) {
        if (navGridData != null) {
            return NavGridLoader.isTraversable(navGridData, x, y);
        }
        
        // If no navgrid, check against obstacles
        for (VisibilityGraph.Obstacle obstacle : staticObstacles) {
            if (obstacle.contains(x, y)) {
                return false;
            }
        }
        
        return isWithinField(x, y);
    }
    
    /**
     * Get navigation grid data (if loaded from navgrid.json)
     */
    public NavGridData getNavGridData() {
        return navGridData;
    }
    
    /**
     * Check if navgrid format is being used
     */
    public boolean hasNavGrid() {
        return navGridData != null;
    }
}