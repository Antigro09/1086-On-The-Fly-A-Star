package pathplanning.control;

import edu.wpi.first.math.geometry.Pose3d;
import pathplanning.config.PlannerConfig;
import pathplanning.field.FieldMap;
import pathplanning.hybrid.HybridAStarPlanner;
import pathplanning.integration.AlgaeTarget;
import pathplanning.robot.RobotState;
import pathplanning.util.PlannerException;
import pathplanning.util.Trajectory;

import java.io.IOException;
import java.nio.file.Path;
import java.time.Clock;
import java.util.List;
import java.util.Objects;

/**
 * High-level planner entry point used by both autonomous and teleoperated flows.
 */
public final class PlannerService {
    private final PlannerConfig config;
    private final FieldMap field;
    private final HybridAStarPlanner planner;
    private final ObjectiveManager objectiveManager;

    public PlannerService(PlannerConfig config, FieldMap field, Clock clock) {
        this.config = Objects.requireNonNull(config);
        this.field = Objects.requireNonNull(field);
        this.planner = new HybridAStarPlanner(config, field);
        this.objectiveManager = new ObjectiveManager(config, clock);
    }

    public static PlannerService fromConfig(Path configPath, Path navgridPath, Clock clock) throws IOException {
        PlannerConfig config = PlannerConfig.load(configPath);
        FieldMap map = FieldMap.load(navgridPath);
        return new PlannerService(config, map, clock);
    }

    public void updateAlgaeDetections(List<AlgaeTarget> detections) {
        objectiveManager.updateAlgaeDetections(detections);
    }

    public Trajectory plan(RobotState robotState, TargetCommand command) throws PlannerException {
        Pose3d start = robotState.pose();
        if (start == null) {
            throw new PlannerException("Robot pose is unknown");
        }
        Pose3d goal = objectiveManager.selectTarget(command, robotState);
        if (goal == null) {
            throw new PlannerException("No goal pose available for request " + command.type());
        }
        return planner.plan(start, goal);
    }

    public PlannerConfig getConfig() {
        return config;
    }

    public FieldMap getField() {
        return field;
    }

    public ObjectiveManager getObjectiveManager() {
        return objectiveManager;
    }
}
