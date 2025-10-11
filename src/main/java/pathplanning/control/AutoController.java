package pathplanning.control;

import edu.wpi.first.math.geometry.Pose3d;
import pathplanning.config.PlannerConfig;

import java.util.List;
import java.util.Objects;

/**
 * Manages autonomous objectives using the routines defined in the configuration file.
 */
public final class AutoController {
    private final PlannerConfig config;
    private PlannerConfig.AutoRoutine activeRoutine;
    private int objectiveIndex;
    private int remainingCount;

    public AutoController(PlannerConfig config) {
        this.config = Objects.requireNonNull(config);
    }

    public void selectRoutine(String routineName) {
        activeRoutine = config.getAutonomous().routines().stream()
                .filter(r -> r.name().equalsIgnoreCase(routineName))
                .findFirst()
                .orElse(null);
        objectiveIndex = 0;
    remainingCount = 0;
    }

    public PlannerConfig.AutoRoutine getActiveRoutine() {
        return activeRoutine;
    }

    public Pose3d activeStartPose() {
        if (activeRoutine == null || activeRoutine.start() == null) {
            return null;
        }
        return activeRoutine.start().toPose3d();
    }

    public boolean hasNextObjective() {
        return activeRoutine != null && objectiveIndex < activeRoutine.objectives().size();
    }

    public TargetCommand currentTargetCommand() {
        if (!hasNextObjective()) {
            return null;
        }
        PlannerConfig.ObjectiveStep step = activeRoutine.objectives().get(objectiveIndex);
        if (remainingCount <= 0) {
            remainingCount = step.count() != null ? Math.max(1, step.count()) : 1;
        }
        return switch (step.type()) {
            case ALGAE -> new TargetCommand(TargetCommand.TargetType.NEAREST_ALGAE, step.label());
            case PROCESSOR -> new TargetCommand(TargetCommand.TargetType.PROCESSOR, step.label());
            case BARGE -> new TargetCommand(TargetCommand.TargetType.BARGE, step.label());
        };
    }

    public void reset() {
        objectiveIndex = 0;
        remainingCount = 0;
    }

    public void markObjectiveComplete() {
        if (!hasNextObjective()) {
            return;
        }
        remainingCount--;
        if (remainingCount <= 0) {
            objectiveIndex++;
            remainingCount = 0;
        }
    }
}
