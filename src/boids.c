#include "boids.h"

#include "raymath.h"

#include <omp.h>
#include <stdio.h>

typedef struct {
    double x_avg, y_avg, z_avg;
    double dx_avg, dy_avg, dz_avg;
    size_t neighboring_boids;
    double dx_close, dy_close, dz_close;
} Accumulator;

void
boids_init(SimulationData *data, SimulationData *data_updated, int num_boids, Params *params)
{
    data->boids = malloc(2 * num_boids * sizeof(Boid));
    data_updated->boids = malloc(2 * num_boids * sizeof(Boid));

    double cell_size = params->visual_range * 1.01f;

    for (size_t i = 0; i < BOIDS_OMP_THREADS; ++i)
    {
        grid_init(&data->grids[i], BOIDS_X_MAX, BOIDS_Y_MAX, BOIDS_Z_MAX, cell_size, num_boids);
        grid_init(&data_updated->grids[i], BOIDS_X_MAX, BOIDS_Y_MAX, BOIDS_Z_MAX, cell_size, num_boids);
    }

    for (size_t i = 0; i < num_boids; ++i)
    {
        Boid *boid = &data->boids[i];
        boid->x = (double)GetRandomValue(BOIDS_X_MIN, BOIDS_X_MAX);
        boid->y = (double)GetRandomValue(BOIDS_Y_MIN, BOIDS_Y_MAX);
        boid->z = (double)GetRandomValue(BOIDS_Z_MIN, BOIDS_Z_MAX);
        boid->dx = (double)GetRandomValue(0, params->max_speed);
        boid->dy = (double)GetRandomValue(0, params->max_speed);
        boid->dz = (double)GetRandomValue(0, params->max_speed);

        grid_insert(&data->grids[i % BOIDS_OMP_THREADS], boid, boid->x, boid->y, boid->z);
    }
}

void
boids_destroy(SimulationData *data, SimulationData *data_updated)
{
    for (size_t i = 0; i < BOIDS_OMP_THREADS; ++i)
    {
        grid_destroy(&data->grids[i]);
        grid_destroy(&data_updated->grids[i]);
    }

    free(data->boids);
    free(data_updated->boids);
}

void
boids_process_cell(GridCell *cell,
                   Boid *boid,
                   Params *params,
                   Accumulator *accumulator)
{
    for (GridCellNode *node = cell->head; node != NULL; node = node->next)
    {
        if (node->data == boid)
        {
            continue;
        }

        Boid *other_boid = node->data;

        // Compute differences in x, y and z coordinates
        double dx = boid->x - other_boid->x;
        double dy = boid->y - other_boid->y;
        double dz = boid->z - other_boid->z;

        if (fabs(dx) > params->visual_range ||
            fabs(dy) > params->visual_range ||
            fabs(dz) > params->visual_range)
        {
            continue;
        }

        double squared_distance = dx * dx + dy * dy + dz * dz;

        // Is squared distance less than the protected range?
        if (squared_distance < params->protected_range * params->protected_range)
        {
            // If so, calculate difference in x/y/z-coordinates to nearfield boid
            accumulator->dx_close += boid->x - other_boid->x;
            accumulator->dy_close += boid->y - other_boid->y;
            accumulator->dz_close += boid->z - other_boid->z;
        }

        // If not in protected range, is the boid in the visual range?
        else if (squared_distance < params->visual_range * params->visual_range)
        {
            // Add other boid's x/y/z-coord and x/y/z vel to accumulator variables
            accumulator->x_avg += other_boid->x;
            accumulator->y_avg += other_boid->y;
            accumulator->z_avg += other_boid->z;
            accumulator->dx_avg += other_boid->dx;
            accumulator->dy_avg += other_boid->dy;
            accumulator->dz_avg += other_boid->dz;

            // Increment number of boids within visual range
            ++accumulator->neighboring_boids;
        }
    }
}

void
boids_run(SimulationData *data,
          SimulationData *data_updated,
          Matrix *transforms,
          int num_boids,
          Params *params,
          double delta_time)
{
    // Clear grids for the next iteration, they are fully rebuilt every time
    for (size_t t = 0; t < BOIDS_OMP_THREADS; ++t)
    {
        grid_clear(&data_updated->grids[t]);
    }

// For every boid . . .
#pragma omp parallel for num_threads(BOIDS_OMP_THREADS)
    for (size_t i = 0; i < num_boids; ++i)
    {
        size_t thread_idx = omp_get_thread_num();

        Boid *boid = &data->boids[i];
        Boid *boid_updated = &data_updated->boids[i];

        // Zero all accumulator variables
        Accumulator accumulator = {0};

        // Process other boids that are potentially in range . . .
        GridKey key_min = grid_get_key(&data->grids[thread_idx],
                                       boid->x - params->visual_range,
                                       boid->y - params->visual_range,
                                       boid->z - params->visual_range);
        GridKey key_max = grid_get_key(&data->grids[thread_idx],
                                       boid->x + params->visual_range,
                                       boid->y + params->visual_range,
                                       boid->z + params->visual_range);

        for (size_t t = 0; t < BOIDS_OMP_THREADS; ++t)
        {
            for (size_t i = key_min.i; i <= key_max.i; ++i)
            {
                for (size_t j = key_min.j; j <= key_max.j; ++j)
                {
                    for (size_t k = key_min.k; k <= key_max.k; ++k)
                    {
                        GridCell *cell = grid_get_cell(&data->grids[t], (GridKey){i, j, k});
                        boids_process_cell(cell, boid, params, &accumulator);
                    }
                }
            }
        }

        // If there were any boids in the visual range . . .
        if (accumulator.neighboring_boids > 0)
        {
            // Divide accumulator variables by number of boids in visual range
            accumulator.x_avg /= accumulator.neighboring_boids;
            accumulator.y_avg /= accumulator.neighboring_boids;
            accumulator.z_avg /= accumulator.neighboring_boids;
            accumulator.dx_avg /= accumulator.neighboring_boids;
            accumulator.dy_avg /= accumulator.neighboring_boids;
            accumulator.dz_avg /= accumulator.neighboring_boids;

            // Add the centering/matching contributions to velocity
            boid_updated->dx = boid->dx +
                               (accumulator.x_avg - boid->x) * params->centering_factor +
                               (accumulator.dx_avg - boid->dx) * params->matching_factor;
            boid_updated->dy = boid->dy +
                               (accumulator.y_avg - boid->y) * params->centering_factor +
                               (accumulator.dy_avg - boid->dy) * params->matching_factor;
            boid_updated->dz = boid->dz +
                               (accumulator.z_avg - boid->z) * params->centering_factor +
                               (accumulator.dz_avg - boid->dz) * params->matching_factor;
        }
        else
        {
            boid_updated->dx = boid->dx;
            boid_updated->dy = boid->dy;
            boid_updated->dz = boid->dz;
        }

        // Add the avoidance contribution to velocity
        boid_updated->dx += accumulator.dx_close * params->avoid_factor;
        boid_updated->dy += accumulator.dy_close * params->avoid_factor;
        boid_updated->dz += accumulator.dz_close * params->avoid_factor;

        // If the boid is near a boundary, make it turn by turnfactor
        if (boid->x < BOIDS_X_MIN)
            boid_updated->dx += params->turn_factor;
        if (boid->x > BOIDS_X_MAX)
            boid_updated->dx -= params->turn_factor;
        if (boid->y < BOIDS_Y_MIN)
            boid_updated->dy += params->turn_factor;
        if (boid->y > BOIDS_Y_MAX)
            boid_updated->dy -= params->turn_factor;
        if (boid->z < BOIDS_Z_MIN)
            boid_updated->dz += params->turn_factor;
        if (boid->z > BOIDS_Z_MAX)
            boid_updated->dz -= params->turn_factor;

        // Calculate the boid's speed
        double squared_speed = boid_updated->dx * boid_updated->dx +
                               boid_updated->dy * boid_updated->dy +
                               boid_updated->dz * boid_updated->dz;

        //  Enforce min and max speeds
        if (squared_speed < params->min_speed)
        {
            boid_updated->dx *= params->min_speed * params->min_speed / squared_speed;
            boid_updated->dy *= params->min_speed * params->min_speed / squared_speed;
            boid_updated->dz *= params->min_speed * params->min_speed / squared_speed;
        }
        if (squared_speed > params->max_speed)
        {
            boid_updated->dx *= params->max_speed * params->max_speed / squared_speed;
            boid_updated->dy *= params->max_speed * params->max_speed / squared_speed;
            boid_updated->dz *= params->max_speed * params->max_speed / squared_speed;
        }

        // Update boid's position
        boid_updated->x = boid->x + boid_updated->dx * delta_time;
        boid_updated->y = boid->y + boid_updated->dy * delta_time;
        boid_updated->z = boid->z + boid_updated->dz * delta_time;

        // Update grid for the next iteration
        grid_insert(&data_updated->grids[thread_idx], boid_updated, boid_updated->x, boid_updated->y, boid_updated->z);

        // Update boid's transform for instanced drawing
        const Vector3 default_direction = {0.0f, 1.0f, 0.0f};
        Vector3 direction = Vector3Normalize((Vector3){boid_updated->dx, boid_updated->dy, boid_updated->dz});
        Vector3 axis = Vector3CrossProduct(default_direction, direction);
        float angle = Vector3Angle(default_direction, direction);
        Matrix rotate = MatrixRotate(axis, angle);
        Matrix translate = MatrixTranslate(boid_updated->x, boid_updated->y, boid_updated->z);

        transforms[i] = MatrixMultiply(rotate, translate);
    }
}

void
boids_draw(Camera3D *camera,
           Mesh *mesh,
           Material *material,
           Matrix *transforms,
           int num_boids)
{
    BeginDrawing();

    ClearBackground(RAYWHITE);

    BeginMode3D(*camera);
    DrawMeshInstanced(*mesh, *material, transforms, num_boids);
    EndMode3D();

    DrawFPS(10, 10);

    EndDrawing();
}
