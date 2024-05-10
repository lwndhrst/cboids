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

    size_t num_cells;                             // Total number of cells in the grid
    size_t num_cells_i, num_cells_j, num_cells_k; // Number of cells in the grid per dimension
    GridCell *cells;                              // Flat 3D array for storing cells
} Grid;

void grid_init(Grid *grid, double x_max, double y_max, double z_max, double cell_size, size_t num_items);
void grid_destroy(Grid *grid);
GridKey grid_get_key(Grid *grid, double x, double y, double z);
GridCell *grid_get_cell(Grid *grid, GridKey key);
void grid_insert(Grid *grid, void *data, double x, double y, double z);
void grid_clear(Grid *grid);
