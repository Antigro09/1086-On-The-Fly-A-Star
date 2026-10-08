package pathplanning.config;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.json.JsonMapper;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * Loads the planner configuration from {@code planner_params.yaml} and exposes structured values.
 */
public final class PlannerConfig {
    private final RobotConfig robot;
    private final PlanningConfig planning;
    private final FieldConfig field;
    private final OptimizationConfig optimization;
    private final VelocityProfileConfig velocityProfile;
    private final NetworkConfig network;
    private final VisionConfig vision;
    private final ObstaclesConfig obstacles;
    private final LoggingConfig logging;
    private final PerformanceConfig performance;
    private final NavGridConfig navgrid;
    private final ObjectiveConfig objectives;
    private final AutonomousConfig autonomous;

    @JsonCreator
    public PlannerConfig(
            @JsonProperty(value = "robot", required = true) RobotConfig robot,
            @JsonProperty(value = "planning", required = true) PlanningConfig planning,
            @JsonProperty(value = "field", required = true) FieldConfig field,
            @JsonProperty(value = "optimization", required = true) OptimizationConfig optimization,
            @JsonProperty(value = "velocity_profile", required = true) VelocityProfileConfig velocityProfile,
            @JsonProperty(value = "network", required = true) NetworkConfig network,
            @JsonProperty(value = "vision", required = true) VisionConfig vision,
            @JsonProperty(value = "obstacles", required = true) ObstaclesConfig obstacles,
            @JsonProperty(value = "logging", required = true) LoggingConfig logging,
            @JsonProperty(value = "performance", required = true) PerformanceConfig performance,
            @JsonProperty(value = "navgrid", required = true) NavGridConfig navgrid,
            @JsonProperty(value = "objectives") ObjectiveConfig objectives,
            @JsonProperty(value = "autonomous") AutonomousConfig autonomous) {
        this.robot = Objects.requireNonNull(robot, "robot");
        this.planning = Objects.requireNonNull(planning, "planning");
        this.field = Objects.requireNonNull(field, "field");
        this.optimization = Objects.requireNonNull(optimization, "optimization");
        this.velocityProfile = Objects.requireNonNull(velocityProfile, "velocityProfile");
        this.network = Objects.requireNonNull(network, "network");
        this.vision = Objects.requireNonNull(vision, "vision");
        this.obstacles = Objects.requireNonNull(obstacles, "obstacles");
        this.logging = Objects.requireNonNull(logging, "logging");
        this.performance = Objects.requireNonNull(performance, "performance");
        this.navgrid = Objects.requireNonNull(navgrid, "navgrid");
        this.objectives = objectives != null ? objectives : ObjectiveConfig.empty();
        this.autonomous = autonomous != null ? autonomous : AutonomousConfig.empty();
    }

    public RobotConfig getRobot() {
        return robot;
    }

    public PlanningConfig getPlanning() {
        return planning;
    }

    public FieldConfig getField() {
        return field;
    }

    public OptimizationConfig getOptimization() {
        return optimization;
    }

    public VelocityProfileConfig getVelocityProfile() {
        return velocityProfile;
    }

    public NetworkConfig getNetwork() {
        return network;
    }

    public VisionConfig getVision() {
        return vision;
    }

    public ObstaclesConfig getObstacles() {
        return obstacles;
    }

    public LoggingConfig getLogging() {
        return logging;
    }

    public PerformanceConfig getPerformance() {
        return performance;
    }

    public NavGridConfig getNavgrid() {
        return navgrid;
    }

    public ObjectiveConfig getObjectives() {
        return objectives;
    }

    public AutonomousConfig getAutonomous() {
        return autonomous;
    }

    public static PlannerConfig load(Path path) throws IOException {
        ObjectMapper mapper = JsonMapper.builder(new YAMLFactory())
                .disable(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES)
                .build();
        try (var reader = Files.newBufferedReader(path)) {
            return mapper.readValue(reader, PlannerConfig.class);
        }
    }

    public record RobotConfig(
            double max_velocity,
            double max_acceleration,
            double max_angular_velocity,
            double max_angular_acceleration,
            double radius,
            double track_width,
            double wheelbase) {
    }

    public record PlanningConfig(
            String algorithm,
            double frequency_hz,
            double deviation_threshold,
            boolean replanning_enabled,
            double heuristic_weight,
            int max_iterations,
            long timeout_ms,
            HybridConfig hybrid,
            double grid_collision_margin) {
    }

    public record HybridConfig(
            double step_size,
            double grid_resolution,
            int heading_divisions,
            double min_turning_radius,
            double max_curvature,
            String curvature_samples,
            boolean allow_reverse,
            boolean allow_holonomic,
            boolean allow_in_place_rotation,
            double holonomic_step,
            double reverse_penalty,
            double turn_penalty,
            double holonomic_penalty,
            double rotation_penalty,
            double rotation_weight,
            double goal_position_tolerance,
            double goal_heading_tolerance_deg,
            double rotation_step_deg,
            int collision_samples) {
    }

    public record FieldConfig(String config_format, String config_path) {
    }

    public record OptimizationConfig(
            boolean smoothing_enabled,
            int bezier_points_per_segment,
            double max_curvature,
            boolean simplification_enabled,
            boolean shortcut_enabled,
            CostWeights cost_weights) {
    }

    public record CostWeights(double distance, double curvature, double obstacle_proximity, double time) {
    }

    public record VelocityProfileConfig(double time_step, double acceleration_limit_factor, double deceleration_distance_buffer) {
    }

    public record NetworkConfig(
            String robot_pose_topic,
            String target_pose_topic,
            String trajectory_topic,
            String vision_table,
            String planner_table,
            double update_rate_hz) {
    }

    public record VisionConfig(
            boolean enabled,
            boolean latency_compensation,
            double max_latency_ms,
            boolean use_field_relative,
            boolean fallback_to_robot_relative,
            List<String> object_types,
            ObjectTracking algae) {
    }

    public record ObjectTracking(
            double radius,
            double min_confidence,
            boolean tracking_enabled,
            long max_age_ms) {
    }

    public record ObstaclesConfig(
            double prediction_time,
            long expiry_time_ms,
            double min_confidence,
            double inflation_radius,
            boolean tracking_enabled) {
    }

    public record LoggingConfig(
            boolean enabled,
            String level,
            boolean log_planning_time,
            boolean log_replanning_triggers,
            boolean log_obstacle_updates) {
    }

    public record PerformanceConfig(
            boolean multi_threading,
            boolean cache_visibility_graph,
            boolean lazy_evaluation,
            boolean early_termination) {
    }

    public record NavGridConfig(
            boolean snap_to_grid,
            boolean diagonal_movement,
            boolean corner_cutting) {
    }

    public record ObjectiveConfig(
            Map<String, List<PoseGoal>> processor,
            Map<String, List<PoseGoal>> barge) {

        public List<PoseGoal> getProcessorGoals(String key) {
            return processor != null ? processor.getOrDefault(key, List.of()) : List.of();
        }

        public List<PoseGoal> getBargeGoals(String key) {
            return barge != null ? barge.getOrDefault(key, List.of()) : List.of();
        }

        private static ObjectiveConfig empty() {
            return new ObjectiveConfig(Collections.emptyMap(), Collections.emptyMap());
        }
    }

    public record AutonomousConfig(List<AutoRoutine> routines) {
        public List<AutoRoutine> routines() {
            return routines != null ? routines : List.of();
        }

        private static AutonomousConfig empty() {
            return new AutonomousConfig(List.of());
        }
    }

    public record AutoRoutine(
            String name,
            PoseGoal start,
            List<ObjectiveStep> objectives) {
    }

    public record ObjectiveStep(
            ObjectiveType type,
            String label,
            Integer count) {
    }

    public enum ObjectiveType {
        ALGAE,
        PROCESSOR,
        BARGE
    }

    public record PoseGoal(
            String name,
            Position position,
            Rotation rotation) {

        public Pose3d toPose3d() {
            Translation3d translation = new Translation3d(position.x(), position.y(), position.z());
            Rotation3d rotation3d = new Rotation3d(rotation.roll(), rotation.pitch(), rotation.yaw());
            return new Pose3d(translation, rotation3d);
        }
    }

    public record Position(double x, double y, double z) {
    }

    public record Rotation(double roll, double pitch, double yaw) {
    }
}
