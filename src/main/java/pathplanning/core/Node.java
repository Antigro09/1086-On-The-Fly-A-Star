package pathplanning.core;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * Represents a node in the pathfinding graph
 */
public class Node {
    private final Translation2d position;
    private final Rotation2d rotation;
    private final List<Node> neighbors;
    private final boolean isTemporary;
    
    public Node(double x, double y) {
        this(x, y, new Rotation2d());
    }
    
    public Node(double x, double y, Rotation2d rotation) {
        this.position = new Translation2d(x, y);
        this.rotation = rotation;
        this.neighbors = new ArrayList<>();
        this.isTemporary = false;
    }
    
    public Node(Translation2d position, Rotation2d rotation) {
        this.position = position;
        this.rotation = rotation;
        this.neighbors = new ArrayList<>();
        this.isTemporary = false;
    }
    
    public Node(Pose2d pose) {
        this.position = pose.getTranslation();
        this.rotation = pose.getRotation();
        this.neighbors = new ArrayList<>();
        this.isTemporary = false;
    }
    
    public Node(Pose2d pose, boolean isTemporary) {
        this.position = pose.getTranslation();
        this.rotation = pose.getRotation();
        this.neighbors = new ArrayList<>();
        this.isTemporary = isTemporary;
    }
    
    public void addNeighbor(Node neighbor) {
        if (!neighbors.contains(neighbor)) {
            neighbors.add(neighbor);
        }
    }
    
    public void removeNeighbor(Node neighbor) {
        neighbors.remove(neighbor);
    }
    
    public List<Node> getNeighbors() {
        return new ArrayList<>(neighbors);
    }
    
    public double getX() {
        return position.getX();
    }
    
    public double getY() {
        return position.getY();
    }
    
    public Translation2d getPosition() {
        return position;
    }
    
    public Rotation2d getRotation() {
        return rotation;
    }
    
    public boolean isTemporary() {
        return isTemporary;
    }
    
    public Pose2d toPose2d() {
        return new Pose2d(position, rotation);
    }
    
    public double distanceTo(Node other) {
        return position.getDistance(other.position);
    }
    
    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (obj == null || getClass() != obj.getClass()) return false;
        Node node = (Node) obj;
        return position.getDistance(node.position) < 0.001 &&
               Math.abs(rotation.minus(node.rotation).getRadians()) < 0.01;
    }
    
    @Override
    public int hashCode() {
        return Objects.hash(
            Math.round(position.getX() * 1000),
            Math.round(position.getY() * 1000),
            Math.round(rotation.getRadians() * 100)
        );
    }
    
    @Override
    public String toString() {
        return String.format("Node(%.2f, %.2f, %.2f°)", 
            position.getX(), position.getY(), rotation.getDegrees());
    }
}