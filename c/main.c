#include "raylib.h"

typedef struct {
    float turn_factor;
    float visual_range;
    float protected_range;
    float centering_factor;
    float avoid_factor;
    float matching_factor;
    float max_speed;
    float min_speed;
    float max_bias;
    float bias_increment;
    float bias_val;
} Params;

int
main(void)
{
    InitWindow(800, 450, "raylib [core] example - basic window");

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

    while (!WindowShouldClose())
    {
        BeginDrawing();
        ClearBackground(RAYWHITE);
        DrawText("Congrats! You created your first window!", 190, 200, 20, LIGHTGRAY);
        EndDrawing();
    }

    CloseWindow();

    return 0;
}
