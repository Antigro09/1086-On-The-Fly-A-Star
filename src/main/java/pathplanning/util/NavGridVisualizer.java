package pathplanning.util;

import pathplanning.util.NavGridLoader.NavGridData;

/**
 * Utility to visualize navigation grid in console
 */
public class NavGridVisualizer {
    
    /**
     * Print grid to console
     */
    public static void printGrid(NavGridData navGrid) {
        System.out.println("\nNavigation Grid Visualization:");
        System.out.println("Legend: . = traversable, # = blocked\n");
        
        boolean[][] grid = navGrid.grid;
        
        // Print column numbers
        System.out.print("   ");
        for (int j = 0; j < grid[0].length; j++) {
            if (j % 5 == 0) {
                System.out.print(j / 10);
            } else {
                System.out.print(" ");
            }
        }
        System.out.println();
        
        System.out.print("   ");
        for (int j = 0; j < grid[0].length; j++) {
            System.out.print(j % 10);
        }
        System.out.println();
        
        // Print grid
        for (int i = 0; i < grid.length; i++) {
            System.out.printf("%2d ", i);
            for (int j = 0; j < grid[0].length; j++) {
                System.out.print(grid[i][j] ? "." : "#");
            }
            System.out.println();
        }
        
        System.out.println();
    }
    
    /**
     * Print grid statistics
     */
    public static void printStats(NavGridData navGrid) {
        boolean[][] grid = navGrid.grid;
        int totalCells = grid.length * grid[0].length;
        int traversableCells = 0;
        
        for (int i = 0; i < grid.length; i++) {
            for (int j = 0; j < grid[0].length; j++) {
                if (grid[i][j]) {
                    traversableCells++;
                }
            }
        }
        
        double traversablePercent = (traversableCells * 100.0) / totalCells;
        
        System.out.println("Navigation Grid Statistics:");
        System.out.println("  Total cells: " + totalCells);
        System.out.println("  Traversable: " + traversableCells + 
                          " (" + String.format("%.1f%%", traversablePercent) + ")");
        System.out.println("  Blocked: " + (totalCells - traversableCells) + 
                          " (" + String.format("%.1f%%", 100 - traversablePercent) + ")");
        System.out.println("  Cell size: " + navGrid.nodeSize + " m");
        System.out.println("  Field area: " + String.format("%.2f m²", 
            navGrid.fieldWidth * navGrid.fieldHeight));
    }
}