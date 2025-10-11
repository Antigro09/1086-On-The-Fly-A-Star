package pathplanning.field;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.json.JsonMapper;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Objects;

/**
 * Represents the discrete navigation grid describing the 2025 field.
 */
public final class FieldMap {
    private final double fieldWidthMeters;
    private final double fieldHeightMeters;
    private final double nodeSizeMeters;
    private final boolean[][] occupancy;

    private FieldMap(double fieldWidthMeters, double fieldHeightMeters, double nodeSizeMeters, boolean[][] occupancy) {
        this.fieldWidthMeters = fieldWidthMeters;
        this.fieldHeightMeters = fieldHeightMeters;
        this.nodeSizeMeters = nodeSizeMeters;
        this.occupancy = occupancy;
    }

    public static FieldMap load(Path path) throws IOException {
        ObjectMapper mapper = JsonMapper.builder()
                .disable(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES)
                .build();
        try (var reader = Files.newBufferedReader(path)) {
            NavGridFile data = mapper.readValue(reader, NavGridFile.class);
            boolean[][] grid = new boolean[data.grid.length][];
            for (int row = 0; row < data.grid.length; row++) {
                grid[row] = data.grid[row].clone();
            }
            return new FieldMap(data.field_size.x, data.field_size.y, data.nodeSizeMeters, grid);
        }
    }

    public double getNodeSizeMeters() {
        return nodeSizeMeters;
    }

    public int getWidth() {
        return occupancy.length == 0 ? 0 : occupancy[0].length;
    }

    public int getHeight() {
        return occupancy.length;
    }

    public double getFieldWidthMeters() {
        return fieldWidthMeters;
    }

    public double getFieldHeightMeters() {
        return fieldHeightMeters;
    }

    public boolean isOccupied(int gridX, int gridY) {
        if (gridY < 0 || gridY >= occupancy.length) {
            return true;
        }
        boolean[] row = occupancy[gridY];
        if (gridX < 0 || gridX >= row.length) {
            return true;
        }
        return row[gridX];
    }

    public boolean isOccupiedMeters(double xMeters, double yMeters) {
        GridCell cell = worldToGrid(xMeters, yMeters);
        return isOccupied(cell.x(), cell.y());
    }

    public boolean isOccupiedMeters(double xMeters, double yMeters, double margin) {
        GridCell center = worldToGrid(xMeters, yMeters);
        int radiusCells = (int) Math.ceil(Math.max(0.0, margin) / nodeSizeMeters);
        for (int dy = -radiusCells; dy <= radiusCells; dy++) {
            for (int dx = -radiusCells; dx <= radiusCells; dx++) {
                int gx = center.x() + dx;
                int gy = center.y() + dy;
                if (isOccupied(gx, gy)) {
                    return true;
                }
            }
        }
        return false;
    }

    public GridCell worldToGrid(double xMeters, double yMeters) {
        int xIndex = (int) Math.floor(xMeters / nodeSizeMeters);
        int yIndex = (int) Math.floor(yMeters / nodeSizeMeters);
        yIndex = clamp(yIndex, 0, occupancy.length - 1);
        xIndex = clamp(xIndex, 0, occupancy[0].length - 1);
        return new GridCell(xIndex, yIndex);
    }

    public boolean inBounds(double xMeters, double yMeters) {
        if (xMeters < 0.0 || yMeters < 0.0) {
            return false;
        }
        return xMeters <= fieldWidthMeters && yMeters <= fieldHeightMeters;
    }

    public double gridToWorldX(int gridX) {
        return (gridX + 0.5) * nodeSizeMeters;
    }

    public double gridToWorldY(int gridY) {
        return (gridY + 0.5) * nodeSizeMeters;
    }

    private static int clamp(int value, int min, int max) {
        if (value < min) {
            return min;
        }
        if (value > max) {
            return max;
        }
        return value;
    }

    public record GridCell(int x, int y) {
    }

    private static final class NavGridFile {
        private final Dimensions field_size;
        private final double nodeSizeMeters;
        private final boolean[][] grid;

        @JsonCreator
        private NavGridFile(
                @JsonProperty(value = "field_size", required = true) Dimensions field_size,
                @JsonProperty(value = "nodeSizeMeters", required = true) double nodeSizeMeters,
                @JsonProperty(value = "grid", required = true) boolean[][] grid) {
            this.field_size = Objects.requireNonNull(field_size);
            this.nodeSizeMeters = nodeSizeMeters;
            this.grid = Objects.requireNonNull(grid);
        }
    }

    private record Dimensions(double x, double y) {
    }
}
