package pathplanning.util;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;

import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

/**
 * Manages configuration parameters for path planning
 */
public class ConfigManager {
    private final Map<String, Object> parameters;
    
    public ConfigManager(String yamlPath) {
        this.parameters = new HashMap<>();
        loadFromYaml(yamlPath);
    }
    
    /**
     * Load parameters from YAML file
     */
    private void loadFromYaml(String yamlPath) {
        try {
            ObjectMapper mapper = new ObjectMapper(new YAMLFactory());
            JsonNode root = mapper.readTree(new File(yamlPath));
            
            parseNode("", root);
            
            System.out.println("Loaded configuration from: " + yamlPath);
        } catch (IOException e) {
            System.err.println("Failed to load config, using defaults: " + e.getMessage());
            loadDefaults();
        }
    }
    
    /**
     * Recursively parse YAML nodes
     */
    private void parseNode(String prefix, JsonNode node) {
        if (node.isObject()) {
            node.fields().forEachRemaining(entry -> {
                String key = prefix.isEmpty() ? entry.getKey() : prefix + "." + entry.getKey();
                parseNode(key, entry.getValue());
            });
        } else if (node.isNumber()) {
            parameters.put(prefix, node.asDouble());
        } else if (node.isBoolean()) {
            parameters.put(prefix, node.asBoolean());
        } else if (node.isTextual()) {
            parameters.put(prefix, node.asText());
        }
    }
    
    /**
     * Load default parameters
     */
    private void loadDefaults() {
        // Robot constraints
        parameters.put("robot.max_velocity", 3.0);
        parameters.put("robot.max_acceleration", 2.0);
        parameters.put("robot.max_angular_velocity", Math.PI);
        parameters.put("robot.radius", 0.4);
        
        // Planning parameters
        parameters.put("planning.frequency_hz", 20);
        parameters.put("planning.deviation_threshold", 0.3);
        parameters.put("planning.replanning_enabled", true);
        parameters.put("planning.heuristic_weight", 1.0);
        
        // Obstacle parameters
        parameters.put("obstacles.prediction_time", 1.0);
        parameters.put("obstacles.expiry_time_ms", 2000);
        parameters.put("obstacles.min_confidence", 0.5);
        
        // Optimization parameters
        parameters.put("optimization.smoothing_enabled", true);
        parameters.put("optimization.bezier_points_per_segment", 10);
        parameters.put("optimization.max_curvature", 2.0);
    }
    
    /**
     * Get parameter as double
     */
    public double getDouble(String key, double defaultValue) {
        Object value = parameters.get(key);
        if (value instanceof Number) {
            return ((Number) value).doubleValue();
        }
        return defaultValue;
    }
    
    /**
     * Get parameter as int
     */
    public int getInt(String key, int defaultValue) {
        Object value = parameters.get(key);
        if (value instanceof Number) {
            return ((Number) value).intValue();
        }
        return defaultValue;
    }
    
    /**
     * Get parameter as boolean
     */
    public boolean getBoolean(String key, boolean defaultValue) {
        Object value = parameters.get(key);
        if (value instanceof Boolean) {
            return (Boolean) value;
        }
        return defaultValue;
    }
    
    /**
     * Get parameter as string
     */
    public String getString(String key, String defaultValue) {
        Object value = parameters.get(key);
        if (value instanceof String) {
            return (String) value;
        }
        return defaultValue;
    }
    
    /**
     * Set parameter
     */
    public void set(String key, Object value) {
        parameters.put(key, value);
    }
    
    /**
     * Print all parameters
     */
    public void printAll() {
        System.out.println("Configuration parameters:");
        parameters.forEach((key, value) -> 
            System.out.println("  " + key + " = " + value)
        );
    }
}