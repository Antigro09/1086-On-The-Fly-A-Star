package pathplanning.control;

import edu.wpi.first.math.geometry.Pose3d;
import pathplanning.config.PlannerConfig;
import pathplanning.integration.AlgaeTarget;
import pathplanning.robot.RobotState;

import java.time.Clock;
import java.time.Duration;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.CopyOnWriteArrayList;

/**
 * Maintains configured and dynamic targets and chooses the most appropriate pose.
 */
public final class ObjectiveManager {
    private final PlannerConfig config;
    private final Clock clock;
    private final List<AlgaeTarget> dynamicAlgae = new CopyOnWriteArrayList<>();

    public ObjectiveManager(PlannerConfig config, Clock clock) {
        this.config = Objects.requireNonNull(config);
        this.clock = Objects.requireNonNull(clock);
    }

    public void updateAlgaeDetections(List<AlgaeTarget> detections) {
        dynamicAlgae.clear();
        if (detections != null) {
            dynamicAlgae.addAll(detections);
        }
    }

    public Pose3d selectTarget(TargetCommand command, RobotState robotState) {
        return switch (command.type()) {
            case NONE -> null;
            case DIRECT -> command.directPose();
            case NEAREST_ALGAE -> findNearestAlgae(robotState).orElse(null);
            case PROCESSOR -> selectConfiguredPose(config.getObjectives().processor(), command.label());
            case BARGE -> selectConfiguredPose(config.getObjectives().barge(), command.label());
        };
    }

    private Optional<Pose3d> findNearestAlgae(RobotState robotState) {
        if (robotState == null || robotState.pose() == null) {
            return Optional.empty();
        }
        Pose3d robotPose = robotState.pose();
        Duration maxAge = Duration.ofMillis(config.getVision().algae().max_age_ms());
        long cutoff = clock.millis() - maxAge.toMillis();
        return dynamicAlgae.stream()
                .filter(a -> a.timestampMillis() >= cutoff)
                .filter(a -> a.confidence() >= config.getVision().algae().min_confidence())
                .min(Comparator.comparingDouble(a -> a.pose().getTranslation().getDistance(robotPose.getTranslation())))
                .map(AlgaeTarget::pose);
    }

    private Pose3d selectConfiguredPose(Map<String, List<PlannerConfig.PoseGoal>> map, String label) {
        if (map == null || map.isEmpty()) {
            return null;
        }
        if (label != null) {
            List<PlannerConfig.PoseGoal> goals = map.get(label);
            if (goals != null && !goals.isEmpty()) {
                return goals.get(0).toPose3d();
            }
        }
        // Fallback to the first defined pose in the map
        for (List<PlannerConfig.PoseGoal> goals : map.values()) {
            if (goals != null && !goals.isEmpty()) {
                return goals.get(0).toPose3d();
            }
        }
        return null;
    }

    public List<Pose3d> configuredProcessorTargets() {
        return flatten(config.getObjectives().processor());
    }

    public List<Pose3d> configuredBargeTargets() {
        return flatten(config.getObjectives().barge());
    }

    private static List<Pose3d> flatten(Map<String, List<PlannerConfig.PoseGoal>> map) {
        if (map == null) {
            return List.of();
        }
        List<Pose3d> poses = new ArrayList<>();
        for (List<PlannerConfig.PoseGoal> goals : map.values()) {
            if (goals == null) {
                continue;
            }
            for (PlannerConfig.PoseGoal goal : goals) {
                poses.add(goal.toPose3d());
            }
        }
        return poses;
    }
}
