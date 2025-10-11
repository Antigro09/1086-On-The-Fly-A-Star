package pathplanning.control;

import pathplanning.integration.AlgaeTarget;

import java.util.List;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Teleoperated planner wrapper. Object detection updates desired targets, but drivers retain veto control.
 */
public final class TeleopController {
    private final AtomicReference<TargetCommand> driverCommand = new AtomicReference<>(new TargetCommand(TargetCommand.TargetType.NEAREST_ALGAE));
    private volatile List<AlgaeTarget> visionTargets = List.of();

    public void setDriverCommand(TargetCommand command) {
        driverCommand.set(Objects.requireNonNull(command));
    }

    public TargetCommand currentCommand() {
        return driverCommand.get();
    }

    public List<AlgaeTarget> visionTargets() {
        return visionTargets;
    }

    public void updateVisionTargets(List<AlgaeTarget> detections) {
        this.visionTargets = detections;
    }
}
