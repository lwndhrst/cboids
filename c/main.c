#include "raylib.h"

#define RAYGUI_IMPLEMENTATION
#include "raygui.h"

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
run_simulation(Boid boids[], Boid boids_updated[], int num_boids, Params *params, double delta_time)
{
    // For every boid . . .
    for (int i = 0; i < num_boids; ++i)
    {
        Boid *boid = &boids[i];
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
            if (i == j)
            {
                continue;
            }

            Boid *other_boid = &boids[j];

            // Compute differences in x and y coordinates
            double dx = boid->x - other_boid->x;
            double dy = boid->y - other_boid->y;

            if (fabs(dx) > params->visual_range || fabs(dy) > params->visual_range)
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
        boid_updated->dx += accumulator.dx_close * params->avoid_factor;
        boid_updated->dy += accumulator.dy_close * params->avoid_factor;

        // If the boid is near an edge, make it turn by turnfactor
        if (boid->y < screen_margin)
            boid_updated->dy += params->turn_factor;
        if (boid->x > screen_width - screen_margin)
            boid_updated->dx -= params->turn_factor;
        if (boid->x < screen_margin)
            boid_updated->dx += params->turn_factor;
        if (boid->y > screen_height - screen_margin)
            boid_updated->dy -= params->turn_factor;

        // Calculate the boid's speed
        double squared_speed = boid_updated->dx * boid_updated->dx +
                               boid_updated->dy * boid_updated->dy;

        //  Enforce min and max speeds
        if (squared_speed < params->min_speed)
        {
            boid_updated->dx *= params->min_speed * params->min_speed / squared_speed;
            boid_updated->dy *= params->min_speed * params->min_speed / squared_speed;
        }
        if (squared_speed > params->max_speed)
        {
            boid_updated->dx *= params->max_speed * params->max_speed / squared_speed;
            boid_updated->dy *= params->max_speed * params->max_speed / squared_speed;
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
    InitWindow(screen_width, screen_height, "Boids in C");

    SetTargetFPS(60);

    GuiLoadStyleDefault();

    Params params = {
        0.2f,    // turn_factor
        40.0f,   // visual_range
        8.0f,    // protected_range
        0.0005f, // centering_factor
        0.05f,   // avoid_factor
        0.05f,   // matching_factor
        6.0f,    // max_speed
        3.0f,    // min_speed
    };

    const int slider_width = 200;
    const int slider_height = 20;
    const int slider_spacing = slider_height + 10;
    const int slider_margin = 120;

    const Rectangle visual_range_slider = {
        slider_margin,
        screen_height - slider_spacing * 5,
        slider_width,
        slider_height};
    const Rectangle protected_range_slider = {
        slider_margin,
        screen_height - slider_spacing * 4,
        slider_width,
        slider_height};
    const Rectangle centering_factor_slider = {
        slider_margin,
        screen_height - slider_spacing * 3,
        slider_width,
        slider_height};
    const Rectangle matching_factor_slider = {
        slider_margin,
        screen_height - slider_spacing * 2,
        slider_width,
        slider_height};
    const Rectangle avoid_factor_slider = {
        slider_margin,
        screen_height - slider_spacing,
        slider_width,
        slider_height};

    float visual_range = params.visual_range;
    float protected_range = params.protected_range;
    float centering_factor = params.centering_factor;
    float matching_factor = params.matching_factor;
    float avoid_factor = params.avoid_factor;

    const size_t num_boids = 1000;
    Boid *boids = malloc(2 * num_boids * sizeof(Boid));
    init_boids(boids, num_boids, &params);

    Boid *boids_current = boids;
    Boid *boids_updated = boids + num_boids;

    while (!WindowShouldClose())
    {
        GuiSlider(visual_range_slider, "visual_range ", "", &visual_range, 0.0f, 80.0f);
        GuiSlider(protected_range_slider, "protected_range ", "", &protected_range, 0.0f, 16.0f);
        GuiSlider(centering_factor_slider, "centering_factor ", "", &centering_factor, 0.0f, 0.001f);
        GuiSlider(matching_factor_slider, "matching_factor ", "", &matching_factor, 0.0f, 0.1f);
        GuiSlider(avoid_factor_slider, "avoid_factor ", "", &avoid_factor, 0.0f, 0.1f);

        params.visual_range = visual_range;
        params.protected_range = protected_range;
        params.centering_factor = centering_factor;
        params.matching_factor = matching_factor;
        params.avoid_factor = avoid_factor;

        run_simulation(boids_current, boids_updated, num_boids, &params, 50.0f * GetFrameTime());
        draw_boids(boids_updated, num_boids);

        boids_current = boids_current == boids ? boids + num_boids : boids;
        boids_updated = boids_updated == boids ? boids + num_boids : boids;
    }

    free(boids);

    CloseWindow();

    return 0;
}
