#include <cmath>
#include <iostream>
#include <list>
#include "detector.hpp"
#include "camera.h"

// #define DEBUG
// #define FULL_SCREEN

void Draw_Frame_Oxyz() {
    float len=10, a=0.2;
    DrawLine3D((Vector3){0.0f, 0.0f, 0.0f}, (Vector3){len*cosf(a), 0.0f, len*sinf(a)}, RED);
    DrawLine3D((Vector3){0.0f, 0.0f, 0.0f}, (Vector3){0.0f, len, 0.0f}, GREEN);
    DrawLine3D((Vector3){0.0f, 0.0f, 0.0f}, (Vector3){-len*sinf(a), 0.0f, len*cosf(a)}, BLUE);
}
void Draw_Trajectory(list<Vector3>& point) {
    std::list<Vector3>::iterator it = point.begin();
    Vector3 posstart, posend;
    posstart = *it;
    it++;
    for (; it != point.end(); ++it) {
        posend = *it;
        DrawLine3D(posstart, posend, GREEN);
        posstart = posend;
    }
}

int main(void) {
    Simulator sim1;
    auto *simIn = new UConstant(&sim1, "simIn");
    auto *simMars = new MarsDetector(&sim1, "simMars");
    auto *simMux = new Mux(&sim1, BusSize(1, 1), "simMux");
    auto *out = new MOutput(&sim1);
    sim1.connectU(simIn, simMux, BusSize(0, 0));
    sim1.connectM(simMux, simMars, 0);
    sim1.connectM(simMars, 0, out);
    simIn->Set_OutValue(0);
    simMars->simIntr->Set_InitialValue(Mat(vecdble{R_MARS+125, 0, 0}));
    simMars->simIntv->Set_InitialValue(Mat(vecdble{-v_USV*sin(0.1), v_USV*cos(0.1), 0}));
    sim1.Set_SimStep(0.01);
    sim1.Initialize();
    cout << "Calculating trajectory......" << endl;
    list<Vector3> Points;
    Mat point(3, 1);
    for (int n=0; n<1000; n++) {
        for (int i = 0; i < 1000; i++)
            sim1.Simulate_OneStep();
        point = simMars->simIntr->Get_OutValue();
        Points.push_back((Vector3){
            float(point.at(0,0)) * 1e-3f, 
            float(point.at(1,0)) * 1e-3f, 
            float(point.at(2,0)) * 1e-3f,
        });
    }
    cout << "Trajectory calculating finished." << endl;

#ifndef DEBUG
    Camera camera;
	SetConfigFlags(FLAG_MSAA_4X_HINT);
    SetTargetFPS(60);
#ifdef FULL_SCREEN
    SetWindowMonitor(1);
    SetConfigFlags(FLAG_FULLSCREEN_MODE);
    InitGraph(0, 0, "RayLib-3D");
#else
    InitGraph(1024, 768, "RayLib-3D");
#endif
	Init_Camera(&camera);
    Vector3 cubePosition = { 0.0f, 0.0f, 0.0f };

    while (!WindowShouldClose()) {
        Update_Camera(&camera);
        BeginDrawing();
            ClearBackground(BLACK);
            BeginMode3D(camera);
                DrawGrid(120, 5);
                DrawSphere((Vector3){0.0f, 0.0f, 0.0f}, R_MARS*1e-3, ORANGE);
                Draw_Frame_Oxyz();
                Draw_Trajectory(Points);
            EndMode3D();
            DrawText(TextFormat("%2i FPS", GetFPS()), 0, 0, 20, LIME);
        EndDrawing();
    }
    CloseGraph();
#endif
    return 0;
}
