package pathplanning.dynamic;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;
import pathplanning.integration.VisionDataReceiver;
import pathplanning.util.Path;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertTrue;

class RealTimeReplannerTest {

    @Test
    void triggersReplanWhenDynamicObstacleBlocksPath() {
        DynamicObstacleManager obstacleManager = new DynamicObstacleManager(0.5);

        VisionDataReceiver.DetectedObject obstacle = new VisionDataReceiver.DetectedObject(
            "cone-1",
            "game_piece",
            0.95,
            3.0,
            1.0,
            0.5,
            0.5,
            0.0
        );
        obstacleManager.updateDynamicObstacles(List.of(obstacle));

        Path path = new Path(List.of(
            new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
            new Pose2d(5.0, 1.0, Rotation2d.fromDegrees(0))
        ));

        RealTimeReplanner replanner = new RealTimeReplanner(10);
        Pose2d currentPose = new Pose2d(1.5, 1.0, Rotation2d.fromDegrees(0));

        assertTrue(
            replanner.shouldReplan(currentPose, path, obstacleManager),
            "Replanner should trigger when a dynamic obstacle blocks the path"
        );
    }
}
