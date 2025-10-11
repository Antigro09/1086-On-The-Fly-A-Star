package pathplanning.control;

import edu.wpi.first.math.geometry.Pose3d;

public record TargetCommand(TargetType type, String label, Pose3d directPose) {
    public TargetCommand(TargetType type) {
        this(type, null, null);
    }

    public TargetCommand(TargetType type, String label) {
        this(type, label, null);
    }

    public static TargetCommand direct(Pose3d pose) {
        return new TargetCommand(TargetType.DIRECT, null, pose);
    }

    public static TargetCommand none() {
        return new TargetCommand(TargetType.NONE, null, null);
    }

    public enum TargetType {
        NONE,
        NEAREST_ALGAE,
        PROCESSOR,
        BARGE,
        DIRECT
    }
}
