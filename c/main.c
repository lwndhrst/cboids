#include "raylib.h"

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

const int screen_width = 800;
const int screen_height = 450;
const int screen_margin = 50;

void
init_boids(Boid *boids, int num_boids, Params *params)
{
    for (int i = 0; i < num_boids; ++i)
    {
        Boid *boid = &boids[i];
        boid->x = (double)GetRandomValue(0, screen_width);
        boid->y = (double)GetRandomValue(0, screen_height);
        boid->dx = 0.0f;
        boid->dy = 0.0f;
    }
}

void
run_simulation(Boid *boids, int num_boids)
{
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
    EndDrawing();
}

int
main(void)
{
    InitWindow(screen_width, screen_height, "raylib [core] example - basic window");

    Params params = {
        0.2f,     // turn_factor
        40.0f,    // visual_range
        8.0f,     // protected_range
        0.0005f,  // centering_factor
        0.05f,    // avoid_factor
        0.05f,    // matching_factor
        6.0f,     // max_speed
        3.0f,     // min_speed
        0.01f,    // max_bias
        0.00004f, // bias_increment
        0.001f,   // bias_val
    };

    const int num_boids = 100;
    Boid *boids = malloc(num_boids * sizeof(Boid));
    init_boids(boids, num_boids, &params);

    while (!WindowShouldClose())
    {
        run_simulation(boids, num_boids);
        draw_boids(boids, num_boids);
    }

    free(boids);

    CloseWindow();

    return 0;
}
