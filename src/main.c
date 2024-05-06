#include "raylib.h"

#define RAYGUI_IMPLEMENTATION
#include "raygui.h"

#include "boids.h"

#include <math.h>
#include <stdlib.h>

#define GLSL_VERSION 330

const int screen_width = 1600;
const int screen_height = 900;

void
draw_boids(Camera3D *camera,
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

void
draw_gui(Params *params)
{
    const int slider_width = 200;
    const int slider_height = 20;
    const int slider_margin = 10;
    const int slider_spacing = slider_height + slider_margin;

    const Rectangle turn_factor_slider = {
        slider_margin,
        screen_height - slider_spacing * 6,
        slider_width,
        slider_height};
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

    float turn_factor = params->turn_factor;
    float visual_range = params->visual_range;
    float protected_range = params->protected_range;
    float centering_factor = params->centering_factor;
    float matching_factor = params->matching_factor;
    float avoid_factor = params->avoid_factor;

    GuiSlider(turn_factor_slider, "", "turn_factor", &turn_factor, 0.0f, 0.2f);
    GuiSlider(visual_range_slider, "", "visual_range", &visual_range, 0.0f, 80.0f);
    GuiSlider(protected_range_slider, "", "protected_range", &protected_range, 0.0f, 16.0f);
    GuiSlider(centering_factor_slider, "", "centering_factor", &centering_factor, 0.0f, 0.001f);
    GuiSlider(matching_factor_slider, "", "matching_factor", &matching_factor, 0.0f, 0.1f);
    GuiSlider(avoid_factor_slider, "", "avoid_factor", &avoid_factor, 0.0f, 0.1f);

    params->turn_factor = turn_factor;
    params->visual_range = visual_range;
    params->protected_range = protected_range;
    params->centering_factor = centering_factor;
    params->matching_factor = matching_factor;
    params->avoid_factor = avoid_factor;
}

int
main(void)
{
    InitWindow(screen_width, screen_height, "Boids in C");

    SetTargetFPS(60);

    Params params = {
        0.1f,    // turn_factor
        40.0f,   // visual_range
        8.0f,    // protected_range
        0.0005f, // centering_factor
        0.05f,   // avoid_factor
        0.05f,   // matching_factor
        6.0f,    // max_speed
        3.0f,    // min_speed
    };

    const size_t num_boids = 2000;
    Boid *boids = malloc(2 * num_boids * sizeof(Boid));
    init_boids(boids, num_boids, &params);

    Boid *boids_current = boids;
    Boid *boids_updated = boids + num_boids;

    Camera3D camera = {0};
    camera.position = (Vector3){0.0f, 0.0f, 0.0f};
    camera.target = (Vector3){0.0f, 0.0f, 1.0f};
    camera.up = (Vector3){0.0f, 1.0f, 0.0f};
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    Mesh mesh = GenMeshCone(1.0f, 4.0f, 3);

    Matrix *transforms = malloc(num_boids * sizeof(Matrix));

    const char *vert =
        "in vec3 vertexPosition;"
        "in mat4 instanceTransform;"
        "uniform mat4 mvp;"
        "void main()"
        "{"
        "    gl_Position = mvp * instanceTransform * vec4(vertexPosition, 1.0);"
        "}";

    const char *frag =
        "out vec4 fragColor;"
        "void main()"
        "{"
        "    fragColor = vec4(0.4, 0.6, 0.7, 1.0);"
        "}";

    Shader shader = LoadShaderFromMemory(vert, frag);
    // Shader shader = LoadShader(TextFormat("c/shader.vert", GLSL_VERSION),
    //                            TextFormat("c/shader.frag", GLSL_VERSION));

    shader.locs[SHADER_LOC_MATRIX_MVP] = GetShaderLocation(shader, "mvp");
    shader.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocationAttrib(shader, "instanceTransform");

    Material material = LoadMaterialDefault();
    material.shader = shader;

    while (!WindowShouldClose())
    {
        draw_gui(&params);

        run_simulation(boids_current,
                       boids_updated,
                       transforms,
                       num_boids,
                       &params,
                       50.0f * GetFrameTime());

        draw_boids(&camera,
                   &mesh,
                   &material,
                   transforms,
                   num_boids);

        boids_current = boids_current == boids ? boids + num_boids : boids;
        boids_updated = boids_updated == boids ? boids + num_boids : boids;
    }

    free(boids);

    CloseWindow();

    return 0;
}
