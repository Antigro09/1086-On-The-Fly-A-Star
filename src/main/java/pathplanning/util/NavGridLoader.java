package pathplanning.util;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import pathplanning.core.VisibilityGraph;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * Loads navigation grid from navgrid.json format
 * This format is commonly used by PathPlanner and other FRC path planning tools
 */
public class NavGridLoader {
    
    /**
     * Load obstacles from navgrid.json format
     * Converts grid cells marked as 'false' into polygon obstacles
     */
    public static NavGridData loadFromFile(String navGridPath) throws IOException {
        ObjectMapper mapper = new ObjectMapper();
        JsonNode root = mapper.readTree(new File(navGridPath));
        
        // Parse field size
        JsonNode fieldSize = root.get("field_size");
        double fieldWidth = fieldSize.get("x").asDouble();
        double fieldHeight = fieldSize.get("y").asDouble();
        
        // Parse node size
        double nodeSizeMeters = root.get("nodeSizeMeters").asDouble();
        
        // Parse grid
        JsonNode gridNode = root.get("grid");
        boolean[][] grid = parseGrid(gridNode);
        
        System.out.println("Loaded navigation grid:");
        System.out.println("  Field size: " + fieldWidth + " x " + fieldHeight + " m");
        System.out.println("  Node size: " + nodeSizeMeters + " m");
        System.out.println("  Grid dimensions: " + grid.length + " x " + grid[0].length);
        
        // Convert grid to obstacles
        List<VisibilityGraph.Obstacle> obstacles = convertGridToObstacles(
            grid, nodeSizeMeters, fieldWidth, fieldHeight
        );
        
        System.out.println("  Generated " + obstacles.size() + " obstacle regions");
        
        return new NavGridData(fieldWidth, fieldHeight, nodeSizeMeters, grid, obstacles);
    }
    
    /**
     * Parse grid array from JSON
     */
    private static boolean[][] parseGrid(JsonNode gridNode) {
        int rows = gridNode.size();
        int cols = gridNode.get(0).size();
        
        boolean[][] grid = new boolean[rows][cols];
        
        for (int i = 0; i < rows; i++) {
            JsonNode row = gridNode.get(i);
            for (int j = 0; j < cols; j++) {
                grid[i][j] = row.get(j).asBoolean();
            }
        }
        
        return grid;
    }
    
    /**
     * Convert grid to obstacles using connected component analysis
     * Groups adjacent blocked cells into larger polygonal obstacles
     */
    private static List<VisibilityGraph.Obstacle> convertGridToObstacles(
        boolean[][] grid, double nodeSize, double fieldWidth, double fieldHeight
    ) {
        List<VisibilityGraph.Obstacle> obstacles = new ArrayList<>();
        
        int rows = grid.length;
        int cols = grid[0].length;
        
        boolean[][] visited = new boolean[rows][cols];
        
        // Find all connected components of blocked cells
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                if (!grid[i][j] && !visited[i][j]) {
                    // Found unvisited blocked cell - flood fill to find connected region
                    List<GridCell> region = floodFill(grid, visited, i, j);
                    
                    // Convert region to obstacle
                    if (!region.isEmpty()) {
                        VisibilityGraph.Obstacle obstacle = createObstacleFromRegion(
                            region, nodeSize, rows, cols
                        );
                        if (obstacle != null) {
                            obstacles.add(obstacle);
                        }
                    }
                }
            }
        }
        
        return obstacles;
    }
    
    /**
     * Flood fill algorithm to find connected blocked cells
     */
    private static List<GridCell> floodFill(boolean[][] grid, boolean[][] visited, 
                                            int startRow, int startCol) {
        List<GridCell> region = new ArrayList<>();
        List<GridCell> queue = new ArrayList<>();
        
        queue.add(new GridCell(startRow, startCol));
        visited[startRow][startCol] = true;
        
        int rows = grid.length;
        int cols = grid[0].length;
        
        while (!queue.isEmpty()) {
            GridCell cell = queue.remove(0);
            region.add(cell);
            
            // Check 4-connected neighbors
            int[][] directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
            
            for (int[] dir : directions) {
                int newRow = cell.row + dir[0];
                int newCol = cell.col + dir[1];
                
                if (newRow >= 0 && newRow < rows && 
                    newCol >= 0 && newCol < cols &&
                    !grid[newRow][newCol] && !visited[newRow][newCol]) {
                    
                    visited[newRow][newCol] = true;
                    queue.add(new GridCell(newRow, newCol));
                }
            }
        }
        
        return region;
    }
    
    /**
     * Create rectangular obstacle from grid region
     */
    private static VisibilityGraph.Obstacle createObstacleFromRegion(
        List<GridCell> region, double nodeSize, int totalRows, int totalCols
    ) {
        if (region.isEmpty()) {
            return null;
        }
        
        // Find bounding box of region
        int minRow = Integer.MAX_VALUE;
        int maxRow = Integer.MIN_VALUE;
        int minCol = Integer.MAX_VALUE;
        int maxCol = Integer.MIN_VALUE;
        
        for (GridCell cell : region) {
            minRow = Math.min(minRow, cell.row);
            maxRow = Math.max(maxRow, cell.row);
            minCol = Math.min(minCol, cell.col);
            maxCol = Math.max(maxCol, cell.col);
        }
        
        // Convert grid coordinates to field coordinates
        // Note: Grid origin is typically top-left, field origin is bottom-left
        double x1 = minCol * nodeSize;
        double x2 = (maxCol + 1) * nodeSize;
        double y1 = (totalRows - maxRow - 1) * nodeSize;
        double y2 = (totalRows - minRow) * nodeSize;
        
        // Create rectangular obstacle
        double[] xPoints = {x1, x2, x2, x1};
        double[] yPoints = {y1, y1, y2, y2};
        
        return new VisibilityGraph.Obstacle(xPoints, yPoints);
    }
    
    /**
     * Check if a position is traversable in the grid
     */
    public static boolean isTraversable(NavGridData navGrid, double x, double y) {
        int col = (int) (x / navGrid.nodeSize);
        int row = navGrid.grid.length - 1 - (int) (y / navGrid.nodeSize);
        
        if (row < 0 || row >= navGrid.grid.length || 
            col < 0 || col >= navGrid.grid[0].length) {
            return false; // Out of bounds
        }
        
        return navGrid.grid[row][col];
    }
    
    /**
     * Helper class for grid cell coordinates
     */
    private static class GridCell {
        final int row;
        final int col;
        
        GridCell(int row, int col) {
            this.row = row;
            this.col = col;
        }
    }
    
    /**
     * Data class holding navigation grid information
     */
    public static class NavGridData {
        public final double fieldWidth;
        public final double fieldHeight;
        public final double nodeSize;
        public final boolean[][] grid;
        public final List<VisibilityGraph.Obstacle> obstacles;
        
        public NavGridData(double fieldWidth, double fieldHeight, double nodeSize,
                          boolean[][] grid, List<VisibilityGraph.Obstacle> obstacles) {
            this.fieldWidth = fieldWidth;
            this.fieldHeight = fieldHeight;
            this.nodeSize = nodeSize;
            this.grid = grid;
            this.obstacles = obstacles;
        }
        
        public int getGridRows() {
            return grid.length;
        }
        
        public int getGridCols() {
            return grid[0].length;
        }
    }
}