#include <cmath>
#include "detector.hpp"
// #include "camera.h"

int main(void) {
    Simulator sim1;
    auto *simIn = new UConstant(&sim1, "simIn");
    auto *simMars = new MarsDetector(&sim1, "simMars");
    auto *simMux = new Mux(&sim1, BusSize(1, 1), "simMux");
    auto *simDeMux = new DeMux(&sim1, BusSize(3, 1), "simDeMux");
    auto **out = new UOutput*[3];
    out[0] = new UOutput(&sim1, "out1");
    out[1] = new UOutput(&sim1, "out2");
    out[2] = new UOutput(&sim1, "out3");
    sim1.connectU(simIn, simMux, BusSize(0, 0));
    sim1.connectM(simMux, simMars, 0);
    sim1.connectM(simMars, 0, simDeMux);
    sim1.connectU(simDeMux, BusSize(0, 0), out[0]);
    sim1.connectU(simDeMux, BusSize(1, 0), out[1]);
    sim1.connectU(simDeMux, BusSize(2, 0), out[2]);
    simMars->simIntr->Set_InitialValue(Mat(vecdble{R_MARS+125, 0, 0}));
    simMars->simIntv->Set_InitialValue(Mat(vecdble{-v_USV*sin(0.1), v_USV*cos(0.1), 0}));
    sim1.Initialize();
    sim1.Simulate();
    sim1.Plot();

    // Camera camera;
	// SetConfigFlags(FLAG_MSAA_4X_HINT);
	// SetConfigFlags(FLAG_FULLSCREEN_MODE);
    // SetTargetFPS(60);
    // InitGraph(0, 0, "RayLib-3D");
	// Init_Camera(&camera);
    // Vector3 cubePosition = { 0.0f, 0.0f, 0.0f };

    // while (!WindowShouldClose()) {
    //     Update_Camera(&camera);
    //     BeginDrawing();
    //         ClearBackground(BLACK);
    //         BeginMode3D(camera);
    //             DrawGrid(120, 5);
    //             DrawSphere((Vector3){ 0.0f, 0.0f, 0.0f }, 2.0f, ORANGE);
    //         EndMode3D();
    //         DrawText(TextFormat("%2i FPS", GetFPS()), 0, 0, 20, LIME);
    //     EndDrawing();
    // }
    // CloseGraph();
    return 0;
}
