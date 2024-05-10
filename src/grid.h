#pragma once

typedef struct {
    void *data; // Data associated with this node
    void *next; // Next cell node in the list
} GridCellNode;

typedef struct {
    GridCellNode *head; // Head of list of cell nodes
    GridCellNode *tail; // Tail of list of cell nodes
} GridCell;

typedef struct {
    size_t i, j, k;
} GridKey;

typedef struct {
    double x_max, y_max, z_max;
    double cell_size;

    size_t num_nodes;        // Stack size
    GridCellNode *nodes;     // Stack for incrementally building lists per cell
    GridCellNode *nodes_top; // Top of stack

    size_t num_cells_x;
    size_t num_cells_y;
    size_t num_cells_z;
    size_t num_cells; // Number of cells in the grid
    GridCell *cells;  // Flat 3D array for storing cells
} Grid;

void grid_init(Grid *grid, double x_max, double y_max, double z_max, double cell_size, size_t num_items);
void grid_destroy(Grid *grid);
GridKey get_grid_key(Grid *grid, double x, double y, double z);
GridCell *get_grid_cell(Grid *grid, GridKey key);
void grid_insert(Grid *grid, void *data, double x, double y, double z);
void grid_clear(Grid *grid);
