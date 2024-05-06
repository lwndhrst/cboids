#pragma once

#include "boids.h"

#include <stdint.h>

typedef struct {
    size_t max_x, max_y, max_z;
    size_t cell_size;
    uintptr_t *data; // This will just store pointers into the original boid array
} SpatialHashGrid;

void init_grid(SpatialHashGrid *grid,
               size_t max_x,
               size_t max_y,
               size_t max_z,
               size_t cell_size,
               size_t num_boids);

// NOTE: If simulation loop stays multi-threaded, probably need to lock here
void insert_into_grid(SpatialHashGrid *grid, Boid *boid);

// NOTE: If simulation loop stays multi-threaded, probably need to lock here
void update_grid(SpatialHashGrid *grid, Boid *boid);

void find_nearby(SpatialHashGrid *grid, Boid *boid, double range);
