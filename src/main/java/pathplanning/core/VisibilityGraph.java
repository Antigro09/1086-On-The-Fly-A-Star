package pathplanning.core;

import pathplanning.util.GeometryUtils;

import java.awt.geom.Line2D;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Visibility graph for pathfinding around obstacles
 */
public class VisibilityGraph {
    private final List<Node> nodes;
    private final List<Edge> edges;
    private final Map<Node, List<Edge>> adjacencyMap;
    private final List<Obstacle> staticObstacles;
    private List<Obstacle> dynamicObstacles;
    
    public VisibilityGraph(List<Obstacle> staticObstacles) {
        this.nodes = new ArrayList<>();
        this.edges = new ArrayList<>();
        this.adjacencyMap = new HashMap<>();
        this.staticObstacles = staticObstacles;
        this.dynamicObstacles = new ArrayList<>();
        
        buildStaticGraph();
    }
    
    /**
     * Build the initial visibility graph from static obstacles
     */
    private void buildStaticGraph() {
        // Add obstacle vertices as nodes
        for (Obstacle obstacle : staticObstacles) {
            for (Node vertex : obstacle.getVertices()) {
                addNode(vertex);
            }
        }
        
        // Connect all visible nodes
        for (int i = 0; i < nodes.size(); i++) {
            for (int j = i + 1; j < nodes.size(); j++) {
                Node n1 = nodes.get(i);
                Node n2 = nodes.get(j);
                
                if (isVisible(n1, n2, getAllObstacles())) {
                    addEdge(new Edge(n1, n2));
                }
            }
        }
    }
    
    /**
     * Update graph with dynamic obstacles
     */
    public void update(List<Obstacle> newDynamicObstacles) {
        this.dynamicObstacles = newDynamicObstacles;
        
        // Remove temporary nodes and their edges
        nodes.removeIf(node -> node.isTemporary());
        edges.removeIf(edge -> edge.getStart().isTemporary() || 
                              edge.getEnd().isTemporary());
        
        // Rebuild adjacency map
        rebuildAdjacencyMap();
    }
    
    /**
     * Add a temporary node (for start/goal positions)
     */
    public void addTemporaryNode(Node node) {
        addNode(node);
        
        // Connect to all visible nodes
        List<Obstacle> allObstacles = getAllObstacles();
        for (Node existingNode : nodes) {
            if (!existingNode.equals(node) && isVisible(node, existingNode, allObstacles)) {
                addEdge(new Edge(node, existingNode));
            }
        }
    }
    
    /**
     * Add a node to the graph
     */
    public void addNode(Node node) {
        if (!nodes.contains(node)) {
            nodes.add(node);
            adjacencyMap.put(node, new ArrayList<>());
        }
    }
    
    /**
     * Add an edge to the graph
     */
    public void addEdge(Edge edge) {
        if (!edges.contains(edge)) {
            edges.add(edge);
            
            Node start = edge.getStart();
            Node end = edge.getEnd();
            
            adjacencyMap.computeIfAbsent(start, k -> new ArrayList<>()).add(edge);
            adjacencyMap.computeIfAbsent(end, k -> new ArrayList<>()).add(edge);
            
            start.addNeighbor(end);
            end.addNeighbor(start);
        }
    }
    
    /**
     * Get all edges connected to a node
     */
    public List<Edge> getEdges(Node node) {
        return adjacencyMap.getOrDefault(node, new ArrayList<>());
    }
    
    /**
     * Check if two nodes are mutually visible (no obstacles in between)
     */
    private boolean isVisible(Node n1, Node n2, List<Obstacle> obstacles) {
        double x1 = n1.getX();
        double y1 = n1.getY();
        double x2 = n2.getX();
        double y2 = n2.getY();
        
        for (Obstacle obstacle : obstacles) {
            if (obstacle.intersectsLine(x1, y1, x2, y2)) {
                return false;
            }
        }
        
        return true;
    }
    
    /**
     * Get all obstacles (static + dynamic)
     */
    private List<Obstacle> getAllObstacles() {
        List<Obstacle> all = new ArrayList<>(staticObstacles);
        all.addAll(dynamicObstacles);
        return all;
    }
    
    /**
     * Rebuild adjacency map from edges
     */
    private void rebuildAdjacencyMap() {
        adjacencyMap.clear();
        for (Node node : nodes) {
            adjacencyMap.put(node, new ArrayList<>());
        }
        
        for (Edge edge : edges) {
            adjacencyMap.get(edge.getStart()).add(edge);
            adjacencyMap.get(edge.getEnd()).add(edge);
        }
    }
    
    public List<Node> getNodes() {
        return new ArrayList<>(nodes);
    }
    
    public List<Edge> getEdges() {
        return new ArrayList<>(edges);
    }
    
    /**
     * Inner class representing an obstacle
     */
    public static class Obstacle {
        private final double[] xPoints;
        private final double[] yPoints;
        private final int nPoints;
        
        public Obstacle(double[] xPoints, double[] yPoints) {
            if (xPoints.length != yPoints.length) {
                throw new IllegalArgumentException("xPoints and yPoints must have same length");
            }
            this.xPoints = xPoints.clone();
            this.yPoints = yPoints.clone();
            this.nPoints = xPoints.length;
        }
        
        public List<Node> getVertices() {
            List<Node> vertices = new ArrayList<>();
            for (int i = 0; i < nPoints; i++) {
                vertices.add(new Node(xPoints[i], yPoints[i]));
            }
            return vertices;
        }
        
        public boolean intersectsLine(double x1, double y1, double x2, double y2) {
            for (int i = 0; i < nPoints; i++) {
                int j = (i + 1) % nPoints;
                if (Line2D.linesIntersect(
                    x1, y1, x2, y2,
                    xPoints[i], yPoints[i], xPoints[j], yPoints[j]
                )) {
                    return true;
                }
            }
            return false;
        }
        
        public boolean contains(double x, double y) {
            return GeometryUtils.pointInPolygon(x, y, xPoints, yPoints);
        }
    }
}