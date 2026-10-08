// Modified 2026-10-08: isolated historical demo; geometric planning and fail-closed validation.
package pathplanning.util;

import edu.wpi.first.math.geometry.Pose3d;

import java.time.Duration;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

/**
 * Historical demo geometric poses and measured solver time. This is not a timed trajectory.
 */
public final class GeometricPath {
    private final List<Pose3d> poses;
    private final Duration planningDurationMeasured;

    public GeometricPath(List<Pose3d> poses, Duration planningDurationMeasured) {
        this.poses = List.copyOf(poses);
        this.planningDurationMeasured = Objects.requireNonNull(planningDurationMeasured);
        if (planningDurationMeasured.isNegative()) {
            throw new IllegalArgumentException("Planning duration must be nonnegative");
        }
    }

    public List<Pose3d> getPoses() {
        return Collections.unmodifiableList(poses);
    }

    public Duration getPlanningDurationMeasured() {
        return planningDurationMeasured;
    }
}
