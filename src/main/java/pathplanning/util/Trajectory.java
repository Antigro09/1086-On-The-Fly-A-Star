package pathplanning.util;

import edu.wpi.first.math.geometry.Pose3d;

import java.time.Duration;
import java.util.Collections;
import java.util.List;

/**
 * Smooth robot path expressed as a sequence of poses.
 */
public final class Trajectory {
    private final List<Pose3d> poses;
    private final Duration nominalDuration;

    public Trajectory(List<Pose3d> poses, Duration nominalDuration) {
        this.poses = List.copyOf(poses);
        this.nominalDuration = nominalDuration;
    }

    public List<Pose3d> getPoses() {
        return Collections.unmodifiableList(poses);
    }

    public Duration getNominalDuration() {
        return nominalDuration;
    }
}
