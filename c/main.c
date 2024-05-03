#include "raylib.h"

#include <math.h>
#include <stdlib.h>

typedef struct {
    double turn_factor;
    double visual_range;
    double protected_range;
    double centering_factor;
    double avoid_factor;
    double matching_factor;
    double max_speed;
    double min_speed;
    double max_bias;
    double bias_increment;
    double bias_val;
} Params;

typedef struct {
    double x, y, dx, dy;
} Boid;

const int screen_width = 1600;
const int screen_height = 900;
const int screen_margin = 100;

void
init_boids(Boid *boids, int num_boids, Params *params)
{
    for (int i = 0; i < num_boids; ++i)
    {
        Boid *boid = &boids[i];

        boid->x = (double)GetRandomValue(0, screen_width);
        boid->y = (double)GetRandomValue(0, screen_height);
        boid->dx = (double)GetRandomValue(0, params->max_speed);
        boid->dy = (double)GetRandomValue(0, params->max_speed);
    }
}

void
run_simulation(Boid *boids_current, Boid *boids_updated, int num_boids, Params *params, double delta_time)
{
    // For every boid . . .
    for (int i = 0; i < num_boids; ++i)
    {
        Boid *boid = &boids_current[i];
        Boid *boid_updated = &boids_updated[i];

        // Zero all accumulator variables
        struct {
            double x_avg, y_avg, dx_avg, dy_avg;
            size_t neighboring_boids;
            double dx_close, dy_close;
        } accumulator = {0};

        // For every other boid in the flock . . .
        for (int j = 0; j < num_boids; ++j)
        {
            Boid *other_boid = &boids_current[i];

            if (boid == other_boid)
            {
                continue;
            }

            // Compute differences in x and y coordinates
            double dx = boid->x - other_boid->x;
            double dy = boid->y - other_boid->y;

            if (fabs(dx) < params->visual_range && fabs(dy) < params->visual_range)
            {
                continue;
            }

            double squared_distance = dx * dx + dy * dy;

            // Is squared distance less than the protected range?
            if (squared_distance < params->protected_range * params->protected_range)
            {
                // If so, calculate difference in x/y-coordinates to nearfield boid
                accumulator.dx_close += boid->x - other_boid->x;
                accumulator.dy_close += boid->y - other_boid->y;
            }

            // If not in protected range, is the boid in the visual range?
            else if (squared_distance < params->visual_range * params->visual_range)
            {
                // Add other boid's x/y-coord and x/y vel to accumulator variables
                accumulator.x_avg += other_boid->x;
                accumulator.y_avg += other_boid->y;
                accumulator.dx_avg += other_boid->dx;
                accumulator.dy_avg += other_boid->dy;

                // Increment number of boids within visual range
                ++accumulator.neighboring_boids;
            }
        }

        // If there were any boids in the visual range . . .
        if (accumulator.neighboring_boids > 0)
        {
            // Divide accumulator variables by number of boids in visual range
            accumulator.x_avg /= accumulator.neighboring_boids;
            accumulator.y_avg /= accumulator.neighboring_boids;
            accumulator.dx_avg /= accumulator.neighboring_boids;
            accumulator.dy_avg /= accumulator.neighboring_boids;

            // Add the centering/matching contributions to velocity
            boid_updated->dx = boid->dx +
                               (accumulator.x_avg - boid->x) * params->centering_factor +
                               (accumulator.dx_avg - boid->dx) * params->matching_factor;

            boid_updated->dy = boid->dy +
                               (accumulator.y_avg - boid->y) * params->centering_factor +
                               (accumulator.dy_avg - boid->dy) * params->matching_factor;
        }
        else
        {
            boid_updated->dx = boid->dx;
            boid_updated->dy = boid->dy;
        }

        // Add the avoidance contribution to velocity
        boid_updated->dx = boid_updated->dx + accumulator.dx_close * params->avoid_factor;
        boid_updated->dy = boid_updated->dy + accumulator.dy_close * params->avoid_factor;

        // If the boid is near an edge, make it turn by turnfactor
        // (this describes a box, will vary based on boundary conditions)
        if (boid->y < screen_margin)
            boid_updated->dy += params->turn_factor;
        if (boid->x > screen_width - screen_margin)
            boid_updated->dx -= params->turn_factor;
        if (boid->x < screen_margin)
            boid_updated->dx += params->turn_factor;
        if (boid->y > screen_height - screen_margin)
            boid_updated->dy -= params->turn_factor;

        // ##############################################################
        // ### ECE 5730 students only - dynamically update bias value ###
        // ##############################################################
        // # biased to right of screen
        // if (boid in scout group 1):
        //     if (boid.vx > 0):
        //         boid.biasval = min(maxbias, boid.biasval + bias_increment)
        //     else:
        //         boid.biasval = max(bias_increment, boid.biasval - bias_increment)
        // # biased to left of screen
        // else if (boid in scout group 2): # biased to left of screen
        //     if (boid.vx < 0):
        //         boid.biasval = min(maxbias, boid.biasval + bias_increment)
        //     else:
        //         boid.biasval = max(bias_increment, boid.biasval - bias_increment)
        // ##############################################################
        //
        // # If the boid has a bias, bias it!
        // # biased to right of screen
        // if (boid in scout group 1):
        //     boid.vx = (1 - boid.biasval)*boid.vx + (boid.biasval * 1)
        // # biased to left of screen
        // else if (boid in scout group 2):
        //     boid.vx = (1 - boid.biasval)*boid.vx + (boid.biasval * (-1))

        // Calculate the boid's speed
        // Slow step! Lookup the "alpha max plus beta min" algorithm
        double speed = sqrtf(boid_updated->dx * boid_updated->dx +
                             boid_updated->dy * boid_updated->dy);

        //  Enforce min and max speeds
        if (speed < params->min_speed)
        {
            boid_updated->dx = (boid_updated->dx / speed) * params->min_speed;
            boid_updated->dy = (boid_updated->dy / speed) * params->min_speed;
        }
        if (speed > params->max_speed)
        {
            boid_updated->dx = (boid_updated->dx / speed) * params->max_speed;
            boid_updated->dy = (boid_updated->dy / speed) * params->max_speed;
        }

        // Update boid's position
        boid_updated->x = boid->x + boid_updated->dx * delta_time;
        boid_updated->y = boid->y + boid_updated->dy * delta_time;
    }
}

void
draw_boids(Boid *boids, int num_boids)
{
    BeginDrawing();
    ClearBackground(RAYWHITE);
    for (int i = 0; i < num_boids; ++i)
    {
        Boid *boid = &boids[i];
        DrawCircle(boid->x, boid->y, 5.0f, PURPLE);
    }
    DrawFPS(10, 10);
    EndDrawing();
}

int
main(void)
{
    InitWindow(screen_width, screen_height, "raylib [core] example - basic window");

    Params default_params = {
        0.2f,    // turn_factor
        40.0f,   // visual_range
        8.0f,    // protected_range
        0.0005f, // centering_factor
        0.05f,   // avoid_factor
        0.05f,   // matching_factor
        6.0f,    // max_speed
        3.0f,    // min_speed
    };

    Params params = {
        0.2f,   // turn_factor
        50.0f,  // visual_range
        8.0f,   // protected_range
        100.0f, // centering_factor
        0.005f, // avoid_factor
        100.0f, // matching_factor
        12.0f,  // max_speed
        6.0f,   // min_speed
    };

    const size_t num_boids = 50;
    Boid *boids = malloc(2 * num_boids * sizeof(Boid));
    init_boids(boids, num_boids, &params);

    Boid *boids_current = boids;
    Boid *boids_updated = boids + num_boids;

    while (!WindowShouldClose())
    {
        run_simulation(boids_current, boids_updated, num_boids, &params, 50.0f * GetFrameTime());
        draw_boids(boids_updated, num_boids);

        boids_current = boids_current == boids ? boids + num_boids : boids;
        boids_updated = boids_updated == boids ? boids + num_boids : boids;
    }

    free(boids);

    CloseWindow();

    return 0;
}
