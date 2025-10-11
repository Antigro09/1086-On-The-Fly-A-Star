package pathplanning.integration;

import edu.wpi.first.math.geometry.Pose3d;

public record AlgaeTarget(Pose3d pose, double confidence, long timestampMillis) {
}
