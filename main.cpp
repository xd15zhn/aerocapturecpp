#include <cmath>
#include <iostream>
#include <list>
#include "spacecraft.hpp"
#include "camera.h"

// #define DEBUG
#define FULL_SCREEN

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
/**********************
仿真部分
**********************/
    Mat initR = Mat(vecdble{MARS_R+USV_H, 0, 0});
    Mat initV = Mat(vecdble{-USV_V*sin(USV_Gamma), USV_V*cos(USV_Gamma), 0});
    double sigmaInit = 1;  // 倾侧角
    Spacecraft spacecraft;
    spacecraft.mssIntr->Set_InitialValue(initR);  // 设置惯性坐标系下探测器初始位置向量
    spacecraft.mssIntv->Set_InitialValue(initV);  // 设置惯性坐标系下探测器初始速度向量
    spacecraft.cnstSigma->Set_OutValue(sigmaInit);  // 设置倾侧角常数输入
    cout << "Calculating trajectory......" << endl;
    list<Vector3> Points;  // 存储轨迹点
    Mat point(3, 1);  // 记录轨迹点
    Mat hmat;  // 高度
    double h;
    double vabs = -1;
    for (int n=0; n<1000; n++) {
        spacecraft.sim1.Simulate_OneStep();  // 仿真一个步长
        point = spacecraft.mssIntr->Get_OutValue();  // 记录轨迹点
        Points.push_back((Vector3){  // 存储轨迹点
            float(point.at(0,0)), 
            float(point.at(1,0)), 
            float(point.at(2,0)),
        });
        hmat = spacecraft.mssIntr->Get_OutValue();
        h = Vector3d(hmat).norm2() - MARS_R;
        // cout << h*1e3 << endl;
        if (h<HMIN) break;
        if (h>H_ATMOS && vabs<0)
        vabs = Vector3d(spacecraft.mssIntv->Get_OutValue()).norm2();
    }
    cout << "\nTrajectory calculating finished." << endl;
    cout << spacecraft.mssIntr->Get_OutValue() << endl;
    cout << vabs << endl;

#ifndef DEBUG
/**********************
3D展示部分
**********************/
    Camera camera;  // 添加相机
	SetConfigFlags(FLAG_MSAA_4X_HINT);  // 4倍反锯齿
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
                DrawGrid(100, 6);
                DrawSphere((Vector3){0.0f, 0.0f, 0.0f}, 2, ORANGE);  // 画3D球
                // Draw_Frame_Oxyz();  // 坐标系测试
                Draw_Trajectory(Points);  // 画3D轨迹
            EndMode3D();
            DrawText(TextFormat("%2i FPS", GetFPS()), 0, 0, 20, LIME);
        EndDrawing();
    }
    CloseGraph();
#endif
    return 0;
}
