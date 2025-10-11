package pathplanning.control;

import edu.wpi.first.math.geometry.Pose3d;
import pathplanning.config.PlannerConfig;
import pathplanning.robot.RobotState;

import java.time.Duration;
import java.time.Instant;
import java.util.Objects;

/**
 * Handles the autonomous/teleop transition timing used during an FRC match.
 */
public final class MatchCoordinator {
    private final AutoController autoController;
    private final TeleopController teleopController;
    private Instant autoEndTime;
    private boolean autoEnabled;

    public MatchCoordinator(PlannerConfig config, TeleopController teleopController) {
        this.autoController = new AutoController(config);
        this.teleopController = Objects.requireNonNull(teleopController);
    }

    public void startAutonomous(String routineName, Duration autoDuration) {
        autoController.selectRoutine(routineName);
        autoController.reset();
        autoEnabled = true;
        autoEndTime = Instant.now().plus(autoDuration);
    }

    public void tick() {
        if (!autoEnabled) {
            return;
        }
        if (Instant.now().isAfter(autoEndTime)) {
            autoEnabled = false;
        }
    }

    public TargetCommand currentCommand(RobotState robotState) {
        if (autoEnabled) {
            TargetCommand command = autoController.currentTargetCommand();
            if (command != null) {
                return command;
            }
            autoEnabled = false;
        }
        return teleopController.currentCommand();
    }

    public boolean isAutoEnabled() {
        return autoEnabled;
    }

    public void markObjectiveComplete() {
        autoController.markObjectiveComplete();
    }

    public Pose3d activeStartPose() {
        return autoController.activeStartPose();
    }
}
