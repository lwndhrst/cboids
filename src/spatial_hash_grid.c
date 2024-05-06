#include "spatial_hash_grid.h"

void
init_grid(SpatialHashGrid *grid,
          size_t max_x,
          size_t max_y,
          size_t max_z,
          size_t cell_size,
          size_t num_boids)
{
    grid->max_x = max_x;
    grid->max_y = max_y;
    grid->max_z = max_z;
    grid->cell_size = cell_size;
}

void
insert_into_grid(SpatialHashGrid *grid, Boid *boid)
{
}

void
update_grid(SpatialHashGrid *grid,
            Boid *boid)
{
}
