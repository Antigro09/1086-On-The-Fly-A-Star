package pathplanning.robot;

import edu.wpi.first.math.geometry.Pose3d;

public record RobotState(Pose3d pose, double linearVelocity, double angularVelocity) {
}
