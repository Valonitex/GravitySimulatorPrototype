/* ============================================================================
 * RaylibDemo teaching scaffold -- NOT your n-body integrator.
 * ----------------------------------------------------------------------------
 * This is a minimal RaylibDemo program with every moving part commented in
 * place, so you can see in isolation:
 *   1. window lifecycle
 *   2. a 3D camera and how it's controlled
 *   3. the render loop shape (BeginDrawing / BeginMode3D brackets)
 *   4. GetFrameTime() and a fixed-timestep accumulator -- the exact pattern
 *      you'll use to call physics::move() at a steady dt regardless of how
 *      fast the screen is actually refreshing
 *   5. drawing N "bodies" each frame straight off a live array, with no
 *      manual GPU buffer management (RaylibDemo handles that internally)
 *
 * The "physics" below is fake: three bodies moving in a circle via sin/cos,
 * just so something actually moves on screen. Delete that block and replace
 * it with calls into your real Body / vectorP / physics:: code. Everything
 * else -- camera setup, loop shape, the draw call -- stays the same.
 * ============================================================================ */

#include "raylib.h"
#include <vector>
#include <cmath>

// Minimal stand-in for what you'd pull out of your own Body class.
// RaylibDemo only ever needs position + radius + color to draw something --
// your real Body's mass/velocity/force/etc never need to leave the physics
// side at all.
struct DemoBody
{
    Vector3 position;
    float   radius;
    Color   color;
};

int main(void)
{
    // ---- 1. WINDOW SETUP ---------------------------------------------------
    // InitWindow is the one call that has to happen before anything else.
    InitWindow(1280, 720, "RaylibDemo scaffold");

    // Caps how fast the render loop tries to spin. This does NOT control
    // your physics rate -- that's handled separately, below.
    SetTargetFPS(60);

    // ---- 2. CAMERA ----------------------------------------------------------
    // Camera3D is a plain struct -- there's no "create camera" function,
    // you fill in every field yourself, on purpose.
    Camera3D camera = { 0 };
    camera.position   = { 0.0f, 25.0f, 35.0f }; // where the eye sits in world space
    camera.target     = { 0.0f, 0.0f, 0.0f };   // the point the eye is looking at
    camera.up         = { 0.0f, 1.0f, 0.0f };   // which world axis is "up" on screen
    camera.fovy       = 45.0f;                  // vertical field of view, degrees
    camera.projection = CAMERA_PERSPECTIVE;     // the other option is CAMERA_ORTHOGRAPHIC

    // ---- 3. FAKE BODIES -------------------------------------------------------
    // Three bodies on circular paths at different radii/speeds -- purely so
    // there's motion to look at. This whole block gets deleted once you wire
    // in your actual simulation.
    std::vector<DemoBody> bodies = {
        { {10.0f, 0.0f, 0.0f},   1.0f, RED   },
        { {0.0f, 0.0f, 16.0f},   1.5f, BLUE  },
        { {-6.0f, 0.0f, -6.0f},  0.8f, GREEN },
    };
    float angle = 0.0f; // radians, advances inside the fixed-step block below

    // ---- 4. FIXED-TIMESTEP ACCUMULATOR -----------------------------------------
    // GetFrameTime() returns how long the LAST frame actually took, in
    // seconds -- it's not constant, it depends on your hardware and what's
    // on screen. Your Velocity Verlet integrator wants a constant dt to stay
    // numerically stable, so instead of feeding GetFrameTime() straight into
    // physics::move(), bank it here and spend it in fixed-size chunks.
    const double physicsDt = 1.0 / 120.0; // matches the dt in your sim file
    double accumulator = 0.0;

    // ---- 5. MAIN LOOP -------------------------------------------------------------
    while (!WindowShouldClose())
    {
        // --- update phase ---
        accumulator += GetFrameTime();

        // Auto-orbits the camera around the target so you get depth
        // perception without wiring up your own mouse controls yet.
        UpdateCamera(&camera, CAMERA_ORBITAL);

        // Spend the banked time in fixed dt-sized steps. THIS loop body is
        // where physics::move(bodys) / physics::checkCol(...) belong -- it
        // can run zero, one, or several times per rendered frame depending
        // on how real frame time and physicsDt happen to line up.
        while (accumulator >= physicsDt)
        {
            angle += 0.5f * (float)physicsDt; // stand-in for "step the simulation"

            bodies[0].position = { 10.0f * cosf(angle),        0.0f, 10.0f * sinf(angle) };
            bodies[1].position = { 16.0f * cosf(angle * 0.6f), 0.0f, 16.0f * sinf(angle * 0.6f) };
            bodies[2].position = { 6.0f  * cosf(-angle * 1.3f), 0.0f, 6.0f  * sinf(-angle * 1.3f) };

            accumulator -= physicsDt;
        }

        // --- draw phase ---
        BeginDrawing();
        ClearBackground(BLACK);

        BeginMode3D(camera);

        DrawGrid(20, 1.0f); // ground-plane reference grid, purely visual

        // The entire "rendering" of your simulation: walk the live array,
        // draw what's there. No buffers, no uploads, no shader binding --
        // RaylibDemo owns all of that internally.
        for (const auto& b : bodies)
        {
            DrawSphere(b.position, b.radius, b.color);
        }

        EndMode3D();

        // Anything outside BeginMode3D/EndMode3D draws in flat 2D screen
        // pixels and ignores the camera entirely -- this is where HUD/UI
        // text goes.
        DrawFPS(10, 10);
        DrawText("orbital camera demo -- swap 'bodies' for your Body vector", 10, 40, 18, RAYWHITE);

        EndDrawing();
    }

    // ---- 6. CLEANUP ---------------------------------------------------------------
    CloseWindow();
    return 0;
}

// ---------------------------------------------------------------
// raylib (C++): a flat plane viewed straight-on, like looking at a
// wall or screen with the camera's view direction normal to it.
// ---------------------------------------------------------------

// ---------------------------------------------------------------
// raylib (C++): a flat plane with a circle drawn on top of it,
// viewed straight-on so the camera's line of sight is normal
// (perpendicular) to the surface — like looking at a wall/screen.
// ---------------------------------------------------------------

#include "raylib.h"   // Core raylib types/functions (Vector3, Camera3D, Draw*, etc.)
#include "rlgl.h"     // Low-level drawing functions (rlBegin/rlVertex3f/rlEnd) — needed
                       // because raylib has no built-in "filled 3D circle" function,
                       // so we build the circle ourselves out of triangles.
#include <cmath>      // For cosf/sinf, used to compute points around the circle

#include "raylib.h"

int main()
{
    InitWindow(800, 450, "Sphere");

    Camera3D camera = { 0 };
    camera.position   = (Vector3){ 6.0f, 6.0f, 6.0f };
    camera.target     = (Vector3){ 0.0f, 0.0f, 0.0f };
    camera.up         = (Vector3){ 0.0f, 1.0f, 0.0f };
    camera.fovy       = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    Vector3 spherePos = { 0.0f, 1.0f, 0.0f };
    float sphereRadius = 1.0f;

    SetTargetFPS(60);

    while (!WindowShouldClose())
    {
        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode3D(camera);
        DrawSphere(spherePos, sphereRadius, RED);
        DrawGrid(10, 1.0f);
        EndMode3D();

        EndDrawing();
    }

    CloseWindow();
    return 0;
}