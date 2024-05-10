#pragma once

#include "grid.h"
#include "raylib.h"

#define BOIDS_X_MIN -50.0f
#define BOIDS_X_MAX 50.0f
#define BOIDS_Y_MIN -50.0f
#define BOIDS_Y_MAX 50.0f
#define BOIDS_Z_MIN 200.0f
#define BOIDS_Z_MAX 300.0f

#define BOIDS_OMP_THREADS 4

typedef struct {
    double turn_factor;
    double visual_range;
    double protected_range;
    double centering_factor;
    double avoid_factor;
    double matching_factor;
    double max_speed;
    double min_speed;
} Params;

typedef struct {
    double x, y, z;
    double dx, dy, dz;
} Boid;

typedef struct {
    Boid *boids;
    Grid grids[BOIDS_OMP_THREADS];
} SimulationData;

void boids_init(SimulationData *data, SimulationData *data_updated, int num_boids, Params *params);
void boids_destroy(SimulationData *data, SimulationData *data_updated);
void boids_run(SimulationData *data, SimulationData *data_updated, Matrix *transforms, int num_boids, Params *params, double delta_time);
void boids_draw(Camera3D *camera, Mesh *mesh, Material *material, Matrix *transforms, int num_boids);
