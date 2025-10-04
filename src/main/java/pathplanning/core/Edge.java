package pathplanning.core;

import java.util.Objects;

/**
 * Represents an edge between two nodes in the pathfinding graph
 */
public class Edge {
    private final Node start;
    private final Node end;
    private final double cost;
    
    public Edge(Node start, Node end) {
        this.start = start;
        this.end = end;
        this.cost = start.distanceTo(end);
    }
    
    public Edge(Node start, Node end, double cost) {
        this.start = start;
        this.end = end;
        this.cost = cost;
    }
    
    public Node getStart() {
        return start;
    }
    
    public Node getEnd() {
        return end;
    }
    
    public double getCost() {
        return cost;
    }
    
    public Node getOther(Node node) {
        if (node.equals(start)) {
            return end;
        } else if (node.equals(end)) {
            return start;
        }
        throw new IllegalArgumentException("Node is not part of this edge");
    }
    
    public boolean contains(Node node) {
        return start.equals(node) || end.equals(node);
    }
    
    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (obj == null || getClass() != obj.getClass()) return false;
        Edge edge = (Edge) obj;
        return (start.equals(edge.start) && end.equals(edge.end)) ||
               (start.equals(edge.end) && end.equals(edge.start));
    }
    
    @Override
    public int hashCode() {
        return Objects.hash(start, end) + Objects.hash(end, start);
    }
    
    @Override
    public String toString() {
        return String.format("Edge(%s -> %s, cost=%.2f)", start, end, cost);
    }
}