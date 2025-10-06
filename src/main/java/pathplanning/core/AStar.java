package pathplanning.core;

import edu.wpi.first.math.geometry.Pose2d;
import pathplanning.util.Path;

import java.util.*;

/**
 * A* pathfinding algorithm implementation
 */
public class AStar {
    private final VisibilityGraph visGraph;
    private final double heuristicWeight;
    
    public AStar(List<VisibilityGraph.Obstacle> staticObstacles) {
        this.visGraph = new VisibilityGraph(staticObstacles);
        this.heuristicWeight = 1.0;
    }
    
    public AStar(List<VisibilityGraph.Obstacle> staticObstacles, double heuristicWeight) {
        this.visGraph = new VisibilityGraph(staticObstacles);
        this.heuristicWeight = heuristicWeight;
    }
    
    /*
     * Find path from start to goal using A* algorithm
     */
    public Path findPath(Pose2d start, Pose2d goal, List<VisibilityGraph.Obstacle> dynamicObstacles) {
        // Update visibility graph with dynamic obstacles
        visGraph.update(dynamicObstacles);
        
        // Create start and goal nodes
        Node startNode = new Node(start, true);
        Node goalNode = new Node(goal, true);
        
        // Add temporary nodes to graph
        visGraph.addTemporaryNode(startNode);
        visGraph.addTemporaryNode(goalNode);
        
        // Run A* search
        List<Node> nodePath = search(startNode, goalNode);
        
        if (nodePath == null) {
            return null;
        }
        
        // Convert node path to Pose2d path
        List<Pose2d> poses = new ArrayList<>();
        for (Node node : nodePath) {
            poses.add(node.toPose2d());
        }
        
        return new Path(poses);
    }
    
    /**
     * A* search algorithm
     */
    private List<Node> search(Node start, Node goal) {
        PriorityQueue<SearchNode> openSet = new PriorityQueue<>(
            Comparator.comparingDouble(n -> n.fCost)
        );
        
        Map<Node, SearchNode> nodeMap = new HashMap<>();
        Set<Node> closedSet = new HashSet<>();
        
        // Initialize start node
        SearchNode startSearch = new SearchNode(start);
        startSearch.gCost = 0;
        startSearch.hCost = heuristic(start, goal);
        startSearch.fCost = startSearch.hCost;
        
        openSet.add(startSearch);
        nodeMap.put(start, startSearch);
        
        int iterations = 0;
        int maxIterations = 10000;
        
        while (!openSet.isEmpty() && iterations < maxIterations) {
            iterations++;
            
            SearchNode current = openSet.poll();
            
            if (current.node.equals(goal)) {
                return reconstructPath(current);
            }
            
            closedSet.add(current.node);
            
            // Explore neighbors
            for (Edge edge : visGraph.getEdges(current.node)) {
                Node neighbor = edge.getOther(current.node);
                
                if (closedSet.contains(neighbor)) {
                    continue;
                }
                
                double tentativeG = current.gCost + edge.getCost();
                
                SearchNode neighborSearch = nodeMap.get(neighbor);
                if (neighborSearch == null) {
                    neighborSearch = new SearchNode(neighbor);
                    nodeMap.put(neighbor, neighborSearch);
                }
                
                if (tentativeG < neighborSearch.gCost) {
                    neighborSearch.parent = current;
                    neighborSearch.gCost = tentativeG;
                    neighborSearch.hCost = heuristic(neighbor, goal);
                    neighborSearch.fCost = neighborSearch.gCost + 
                                          heuristicWeight * neighborSearch.hCost;
                    
                    if (!openSet.contains(neighborSearch)) {
                        openSet.add(neighborSearch);
                    }
                }
            }
        }
        
        // No path found
        return null;
    }
    
    /**
     * Heuristic function (Euclidean distance with rotation penalty)
     */
    private double heuristic(Node a, Node b) {
        double distance = a.distanceTo(b);
        
        // Add rotational cost
        double rotationDiff = Math.abs(
            a.getRotation().minus(b.getRotation()).getRadians()
        );
        double rotationCost = rotationDiff * 0.5;
        
        return distance + rotationCost;
    }
    
    /**
     * Reconstruct path from goal to start
     */
    private List<Node> reconstructPath(SearchNode goalNode) {
        List<Node> path = new ArrayList<>();
        SearchNode current = goalNode;
        
        while (current != null) {
            path.add(0, current.node);
            current = current.parent;
        }
        
        return path;
    }
    
    /**
     * Internal class for A* search
     */
    private static class SearchNode {
        Node node;
        SearchNode parent;
        double gCost = Double.POSITIVE_INFINITY;
        double hCost = 0;
        double fCost = Double.POSITIVE_INFINITY;
        
        SearchNode(Node node) {
            this.node = node;
        }
    }
}