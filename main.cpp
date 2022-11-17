#include <cmath>
#include "detector.hpp"
#include "camera.h"

Mat simFUNCTIONf1(Mat *u) {

}

int main(void) {
    Simulator sim1;

    Camera camera;
	SetConfigFlags(FLAG_MSAA_4X_HINT);
	SetConfigFlags(FLAG_FULLSCREEN_MODE);
    SetTargetFPS(60);
    InitGraph(0, 0, "RayLib-3D");
	Init_Camera(&camera);
    Vector3 cubePosition = { 0.0f, 0.0f, 0.0f };

    while (!WindowShouldClose()) {
        Update_Camera(&camera);
        BeginDrawing();
            ClearBackground(BLACK);
            BeginMode3D(camera);
                DrawGrid(120, 5);
                DrawSphere((Vector3){ 0.0f, 0.0f, 0.0f }, 2.0f, ORANGE);
            EndMode3D();
            DrawText(TextFormat("%2i FPS", GetFPS()), 0, 0, 20, LIME);
        EndDrawing();
    }
    CloseGraph();
    return 0;
}
