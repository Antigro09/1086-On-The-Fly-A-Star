package pathplanning.examples;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import pathplanning.control.PlannerService;
import pathplanning.control.TargetCommand;
import pathplanning.integration.AlgaeTarget;
import pathplanning.robot.RobotState;
import pathplanning.util.PlannerException;
import pathplanning.util.Trajectory;

import java.io.IOException;
import java.nio.file.Path;
import java.time.Clock;
import java.time.Duration;
import java.util.List;

/**
 * Demonstrates the planner with synthetic data so it can be validated without hardware attached.
 */
public final class HybridPlannerExample {
    public static void main(String[] args) throws IOException, PlannerException {
        Path configPath = Path.of("config", "planner_params.yaml");
        Path navgridPath = Path.of("config", "navgrid.json");

        PlannerService plannerService = PlannerService.fromConfig(configPath, navgridPath, Clock.systemUTC());

        RobotState robotState = new RobotState(new Pose3d(1.5, 1.0, 0.0, new Rotation3d(0.0, 0.0, 0.0)), 0.0, 0.0);

        plannerService.updateAlgaeDetections(List.of(
                new AlgaeTarget(new Pose3d(new Translation3d(4.0, 2.0, 0.0), new Rotation3d()), 0.8, System.currentTimeMillis()),
                new AlgaeTarget(new Pose3d(new Translation3d(6.0, 3.0, 0.0), new Rotation3d()), 0.9, System.currentTimeMillis())
        ));

        runScenario(plannerService, robotState, new TargetCommand(TargetCommand.TargetType.NEAREST_ALGAE));
        runScenario(plannerService, robotState, new TargetCommand(TargetCommand.TargetType.PROCESSOR, "default"));
        runScenario(plannerService, robotState, new TargetCommand(TargetCommand.TargetType.BARGE, "default"));
    }

    private static void runScenario(PlannerService plannerService, RobotState state, TargetCommand command) throws PlannerException {
        Trajectory trajectory = plannerService.plan(state, command);
        System.out.println("=== Scenario: " + command.type() + " => poses: " + trajectory.getPoses().size());
        trajectory.getPoses().stream()
                .limit(5)
                .forEach(pose -> System.out.printf("  (%.2f, %.2f) heading=%.1f deg%n",
                        pose.getX(), pose.getY(), Math.toDegrees(pose.getRotation().getZ())));
        if (trajectory.getPoses().size() > 5) {
            System.out.println("  ...");
        }
        System.out.println("  Nominal duration: " + trajectory.getNominalDuration().toMillis() + " ms");
        System.out.println();
    }
}
