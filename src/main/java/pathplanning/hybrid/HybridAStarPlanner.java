package pathplanning.hybrid;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import pathplanning.config.PlannerConfig;
import pathplanning.field.FieldMap;
import pathplanning.util.PlannerException;
import pathplanning.util.Trajectory;

import java.time.Duration;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.PriorityQueue;
import java.util.Set;

/**
 * Basic Hybrid A* implementation that supports forward/backwards, strafing, and in-place rotations.
 * The search space is discretised by a configurable position/heading resolution.
 */
public final class HybridAStarPlanner {
    private final PlannerConfig config;
    private final FieldMap field;

    public HybridAStarPlanner(PlannerConfig config, FieldMap field) {
        this.config = Objects.requireNonNull(config);
        this.field = Objects.requireNonNull(field);
    }

    public Trajectory plan(Pose3d start, Pose3d goal) throws PlannerException {
        PlannerConfig.HybridConfig hybrid = config.getPlanning().hybrid();
        int headingDivisions = hybrid.heading_divisions();
        double headingResolution = 2.0 * Math.PI / headingDivisions;
        double stepSize = hybrid.step_size();
        double holonomicStep = hybrid.holonomic_step();
        double rotationStepRad = Math.toRadians(hybrid.rotation_step_deg());

        Node startNode = Node.fromPose(start, field, headingDivisions);
        Node goalNode = Node.fromPose(goal, field, headingDivisions);

        PriorityQueue<Node> openSet = new PriorityQueue<>(Comparator.comparingDouble(Node::fScore));
        Map<NodeKey, Node> bestNodes = new HashMap<>();
        Set<NodeKey> closed = new HashSet<>();

        startNode.gScore = 0.0;
        startNode.hScore = heuristic(startNode, goalNode, hybrid);
        openSet.add(startNode);
        bestNodes.put(startNode.key(), startNode);

        long maxIterations = Math.max(1, config.getPlanning().max_iterations());
        long iterations = 0;
        long timeoutMillis = config.getPlanning().timeout_ms();
        long deadline = System.currentTimeMillis() + timeoutMillis;

        while (!openSet.isEmpty()) {
            if (iterations++ > maxIterations) {
                throw new PlannerException("Hybrid A* exceeded iteration budget");
            }
            if (System.currentTimeMillis() > deadline) {
                throw new PlannerException("Hybrid A* timed out");
            }
            Node current = openSet.poll();
            if (closed.contains(current.key())) {
                continue;
            }
            closed.add(current.key());

            if (reachedGoal(current, goalNode, hybrid)) {
                return reconstruct(current, goal);
            }

            for (Node neighbor : expandNeighbors(current, headingResolution, stepSize, holonomicStep, rotationStepRad, hybrid)) {
                if (closed.contains(neighbor.key())) {
                    continue;
                }
                double tentative = current.gScore + transitionCost(current, neighbor, hybrid);
                Node existing = bestNodes.get(neighbor.key());
                if (existing != null && tentative >= existing.gScore) {
                    continue;
                }
                if (collisionAlongSegment(current, neighbor, hybrid)) {
                    continue;
                }
                neighbor.parent = current;
                neighbor.gScore = tentative;
                neighbor.hScore = heuristic(neighbor, goalNode, hybrid);
                bestNodes.put(neighbor.key(), neighbor);
                openSet.add(neighbor);
            }
        }
        throw new PlannerException("Hybrid A* failed: no path to goal");
    }

    private boolean collisionAlongSegment(Node start, Node end, PlannerConfig.HybridConfig hybrid) {
        int samples = Math.max(2, hybrid.collision_samples());
        double step = 1.0 / (samples - 1);
        double margin = config.getPlanning().grid_collision_margin() + config.getRobot().radius();
        for (int i = 0; i < samples; i++) {
            double t = i * step;
            double x = lerp(start.xMeters, end.xMeters, t);
            double y = lerp(start.yMeters, end.yMeters, t);
            if (!field.inBounds(x, y)) {
                return true;
            }
            if (field.isOccupiedMeters(x, y, margin)) {
                return true;
            }
        }
        return false;
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    private static boolean reachedGoal(Node node, Node goal, PlannerConfig.HybridConfig hybrid) {
        double positionTol = hybrid.goal_position_tolerance();
        double headingTol = Math.toRadians(hybrid.goal_heading_tolerance_deg());
        double dx = node.xMeters - goal.xMeters;
        double dy = node.yMeters - goal.yMeters;
        double distance = Math.hypot(dx, dy);
        double headingDiff = Math.abs(wrapAngle(node.headingRad - goal.headingRad));
        return distance <= positionTol && headingDiff <= headingTol;
    }

    private double heuristic(Node node, Node goal, PlannerConfig.HybridConfig hybrid) {
        double distance = Math.hypot(node.xMeters - goal.xMeters, node.yMeters - goal.yMeters);
        double headingDiff = Math.abs(wrapAngle(node.headingRad - goal.headingRad));
        double weightedHeading = headingDiff * hybrid.rotation_weight();
        return (distance + weightedHeading) * config.getPlanning().heuristic_weight();
    }

    private double transitionCost(Node from, Node to, PlannerConfig.HybridConfig hybrid) {
        double distance = Math.hypot(from.xMeters - to.xMeters, from.yMeters - to.yMeters);
        double reversePenalty = to.isReverse ? hybrid.reverse_penalty() : 1.0;
        double holonomicPenalty = to.isHolonomic ? hybrid.holonomic_penalty() : 1.0;
        double rotationPenalty = to.isRotation ? hybrid.rotation_penalty() : 1.0;
        double turnPenalty = Math.abs(wrapAngle(to.headingRad - from.headingRad)) > 1e-6
                ? hybrid.turn_penalty() : 1.0;
        return distance * reversePenalty * holonomicPenalty * turnPenalty + rotationPenalty;
    }

    private List<Node> expandNeighbors(
            Node current,
            double headingResolution,
            double stepSize,
            double holonomicStep,
            double rotationStep,
            PlannerConfig.HybridConfig hybrid) {
        List<Node> neighbors = new ArrayList<>();
        // Forward/Reverse moves
        double heading = current.headingRad;
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        addNeighbor(current, neighbors, current.xMeters + cos * stepSize, current.yMeters + sin * stepSize, heading,
                false, false, false, hybrid);
        if (hybrid.allow_reverse()) {
            addNeighbor(current, neighbors, current.xMeters - cos * stepSize, current.yMeters - sin * stepSize, heading,
                    true, false, false, hybrid);
        }
        if (hybrid.allow_holonomic()) {
            double leftX = current.xMeters - sin * holonomicStep;
            double leftY = current.yMeters + cos * holonomicStep;
            double rightX = current.xMeters + sin * holonomicStep;
            double rightY = current.yMeters - cos * holonomicStep;
            addNeighbor(current, neighbors, leftX, leftY, heading, false, true, false, hybrid);
            addNeighbor(current, neighbors, rightX, rightY, heading, false, true, false, hybrid);
        }
        if (hybrid.allow_in_place_rotation()) {
            addNeighbor(current, neighbors, current.xMeters, current.yMeters,
                    wrapAngle(current.headingRad + rotationStep), false, false, true, hybrid);
            addNeighbor(current, neighbors, current.xMeters, current.yMeters,
                    wrapAngle(current.headingRad - rotationStep), false, false, true, hybrid);
        }
        // Turning moves to approximate curvature changes
        if (!hybrid.allow_holonomic()) {
            double minRadius = Math.max(stepSize, hybrid.min_turning_radius());
            double turnAngle = stepSize / minRadius;
            double leftHeading = wrapAngle(current.headingRad + turnAngle);
            double rightHeading = wrapAngle(current.headingRad - turnAngle);
            double leftX = current.xMeters + Math.cos(leftHeading) * stepSize;
            double leftY = current.yMeters + Math.sin(leftHeading) * stepSize;
            double rightX = current.xMeters + Math.cos(rightHeading) * stepSize;
            double rightY = current.yMeters + Math.sin(rightHeading) * stepSize;
            addNeighbor(current, neighbors, leftX, leftY, leftHeading, false, false, false, hybrid);
            addNeighbor(current, neighbors, rightX, rightY, rightHeading, false, false, false, hybrid);
        }
        return neighbors;
    }

    private void addNeighbor(
            Node current,
            List<Node> neighbors,
            double xMeters,
            double yMeters,
            double heading,
            boolean reverse,
            boolean holonomic,
            boolean rotation,
            PlannerConfig.HybridConfig hybrid) {
        if (!field.inBounds(xMeters, yMeters)) {
            return;
        }
        FieldMap.GridCell cell = field.worldToGrid(xMeters, yMeters);
        if (field.isOccupied(cell.x(), cell.y())) {
            return;
        }
        Node neighbor = new Node(cell.x(), cell.y(), headingToIndex(heading, hybrid.heading_divisions()));
        neighbor.xMeters = xMeters;
        neighbor.yMeters = yMeters;
        neighbor.headingRad = heading;
        neighbor.isReverse = reverse;
        neighbor.isHolonomic = holonomic;
        neighbor.isRotation = rotation;
        neighbors.add(neighbor);
    }

    private Trajectory reconstruct(Node goalNode, Pose3d goalPose) {
        List<Pose3d> poses = new ArrayList<>();
        Node current = goalNode;
        while (current != null) {
            Pose3d pose = new Pose3d(
                    new Translation3d(current.xMeters, current.yMeters, goalPose.getTranslation().getZ()),
                    new Rotation3d(0.0, 0.0, current.headingRad));
            poses.add(pose);
            current = current.parent;
        }
        List<Pose3d> ordered = new ArrayList<>(poses.size());
        for (int i = poses.size() - 1; i >= 0; i--) {
            ordered.add(poses.get(i));
        }
        return new Trajectory(ordered, Duration.ofMillis(config.getPlanning().timeout_ms()));
    }

    private static double wrapAngle(double angle) {
        double wrapped = angle % (2.0 * Math.PI);
        if (wrapped > Math.PI) {
            wrapped -= 2.0 * Math.PI;
        } else if (wrapped < -Math.PI) {
            wrapped += 2.0 * Math.PI;
        }
        return wrapped;
    }

    private int headingToIndex(double heading, int divisions) {
        double normalized = (heading % (2.0 * Math.PI) + 2.0 * Math.PI) % (2.0 * Math.PI);
        int index = (int) Math.round(normalized / (2.0 * Math.PI) * divisions) % divisions;
        return Math.max(0, Math.min(divisions - 1, index));
    }

    private static final class Node {
        private final int gridX;
        private final int gridY;
        private final int headingIndex;
        private Node parent;
        private double xMeters;
        private double yMeters;
        private double headingRad;
        private double gScore = Double.POSITIVE_INFINITY;
        private double hScore = Double.POSITIVE_INFINITY;
        private boolean isReverse;
        private boolean isHolonomic;
        private boolean isRotation;

        private Node(int gridX, int gridY, int headingIndex) {
            this.gridX = gridX;
            this.gridY = gridY;
            this.headingIndex = headingIndex;
        }

        private static Node fromPose(Pose3d pose, FieldMap field, int headingDivisions) {
            FieldMap.GridCell cell = field.worldToGrid(pose.getX(), pose.getY());
            double heading = pose.getRotation().getZ();
            int headingIndex = (int) Math.round(((heading % (2.0 * Math.PI)) + 2.0 * Math.PI) % (2.0 * Math.PI) / (2.0 * Math.PI) * headingDivisions) % headingDivisions;
            Node node = new Node(cell.x(), cell.y(), headingIndex);
            node.xMeters = pose.getX();
            node.yMeters = pose.getY();
            node.headingRad = heading;
            return node;
        }

        private NodeKey key() {
            return new NodeKey(gridX, gridY, headingIndex);
        }

        private double fScore() {
            return gScore + hScore;
        }
    }

    private record NodeKey(int x, int y, int heading) {
    }
}
