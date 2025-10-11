package pathplanning.integration;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import pathplanning.control.MatchCoordinator;
import pathplanning.control.PlannerService;
import pathplanning.control.TargetCommand;
import pathplanning.control.TeleopController;
import pathplanning.robot.RobotState;
import pathplanning.util.PlannerException;
import pathplanning.util.Trajectory;

import java.io.IOException;
import java.nio.file.Path;
import java.time.Clock;
import java.time.Duration;
import java.time.Instant;
import java.util.List;
import java.util.Optional;

/**
 * Entry point intended to run on the Jetson. Integrates operator commands, RoboRIO pose updates, and vision targets.
 */
public final class OnTheFlyPlannerMain {
    private final PlannerService plannerService;
    private final TeleopController teleopController;
    private final MatchCoordinator matchCoordinator;

    private RobotState latestRobotState = new RobotState(new Pose3d(), 0.0, 0.0);
    private TargetCommand lastCommand = TargetCommand.none();

    public OnTheFlyPlannerMain(Path configPath, Path navgridPath, Clock clock) throws IOException {
        this.plannerService = PlannerService.fromConfig(configPath, navgridPath, clock);
        this.teleopController = new TeleopController();
        this.matchCoordinator = new MatchCoordinator(plannerService.getConfig(), teleopController);
    }

    public void updateRobotState(RobotState state) {
        this.latestRobotState = state;
    }

    public void updateVision(List<AlgaeTarget> detections) {
        teleopController.updateVisionTargets(detections);
        plannerService.updateAlgaeDetections(detections);
    }

    public void setDriverCommand(TargetCommand command) {
        teleopController.setDriverCommand(command);
    }

    public void requestNearestAlgae() {
        setDriverCommand(new TargetCommand(TargetCommand.TargetType.NEAREST_ALGAE));
    }

    public void requestProcessor(String label) {
        setDriverCommand(new TargetCommand(TargetCommand.TargetType.PROCESSOR, label));
    }

    public void requestBarge(String label) {
        setDriverCommand(new TargetCommand(TargetCommand.TargetType.BARGE, label));
    }

    public void clearRequest() {
        setDriverCommand(TargetCommand.none());
    }

    public void startAutonomous(String routineName, Duration autoDuration) {
        matchCoordinator.startAutonomous(routineName, autoDuration);
    }

    public Optional<Trajectory> planCurrent() throws PlannerException {
        matchCoordinator.tick();
        TargetCommand command = matchCoordinator.currentCommand(latestRobotState);
        if (command == null || command.type() == TargetCommand.TargetType.NONE) {
            return Optional.empty();
        }
        lastCommand = command;
        try {
            return Optional.ofNullable(plannerService.plan(latestRobotState, command));
        } catch (PlannerException ex) {
            if (ex.getMessage() != null && ex.getMessage().startsWith("No goal pose available")) {
                return Optional.empty();
            }
            throw ex;
        }
    }

    public void markObjectiveComplete() {
        matchCoordinator.markObjectiveComplete();
    }

    public static void main(String[] args) throws IOException {
        Path configPath = Path.of("config", "planner_params.yaml");
        Path navgridPath = Path.of("config", "navgrid.json");
        OnTheFlyPlannerMain planner = new OnTheFlyPlannerMain(configPath, navgridPath, Clock.systemUTC());

        planner.startAutonomous("two-algae-barge", Duration.ofSeconds(15));
        planner.updateRobotState(new RobotState(new Pose3d(1.5, 1.0, 0.0, new Rotation3d()), 0.0, 0.0));
        planner.updateVision(List.of());

        Instant end = Instant.now().plusSeconds(17);
        while (Instant.now().isBefore(end)) {
            try {
                Optional<Trajectory> maybePath = planner.planCurrent();
                if (maybePath.isPresent()) {
                    Trajectory path = maybePath.get();
                    System.out.println("Planned " + path.getPoses().size() + " poses towards " + planner.lastCommand().type());
                    planner.markObjectiveComplete();
                } else {
                    System.out.println("Planner idle; no command queued");
                }
            } catch (PlannerException ex) {
                System.err.println("Planning failed: " + ex.getMessage());
            }
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
        }
    }

    public PlannerService plannerService() {
        return plannerService;
    }

    public TeleopController teleopController() {
        return teleopController;
    }

    public MatchCoordinator matchCoordinator() {
        return matchCoordinator;
    }

    public RobotState latestRobotState() {
        return latestRobotState;
    }

    public TargetCommand lastCommand() {
        return lastCommand;
    }

    public Pose3d activeAutonomousStart() {
        return matchCoordinator.activeStartPose();
    }
}
