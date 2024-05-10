#pragma once

#include "raylib.h"

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

void init_boids(Boid *boids, int num_boids, Params *params);
void run_simulation(Boid boids[], Boid boids_updated[], Matrix *transforms, int num_boids, Params *params, double delta_time);
