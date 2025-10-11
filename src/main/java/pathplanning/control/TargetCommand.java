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

    public enum TargetType {
        NEAREST_ALGAE,
        PROCESSOR,
        BARGE,
        DIRECT
    }
}
