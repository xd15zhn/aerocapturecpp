#include <cmath>
#include <iostream>
#include <list>
#include "detector.hpp"
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
    Simulator sim1;  // 添加仿真器(#include "simucpp.hpp")
    auto *simIn = new UInput(&sim1, "simIn");  // 往仿真器中添加输入模块
    auto *simMars = new MarsDetector(&sim1, "simMars");  // 往仿真器中添加被控对象子模块:火星探测器
    auto *simMux = new Mux(&sim1, BusSize(1, 1), "simMux");  // 往仿真器中添加总线复用模块，将倾侧角输入的标量信号转换为矩阵信号
    // auto *out = new MOutput(&sim1);  // 往仿真器中添加输出模块，用于展示波形图。若展示3D轨迹则不需要
    sim1.connectU(simIn, simMux, BusSize(0, 0));  // 连接输入模块与总线复用模块
    sim1.connectM(simMux, simMars, 0);  // 连接总线复用模块与被控对象子模块
    // sim1.connectM(simMars, 0, out);  // 连接被控对象子模块与输出模块
    simIn->Set_Function([](double u){return 1;});  // 输入模块设置倾侧角输入函数
    simMars->simIntr->Set_InitialValue(initR);  // 设置惯性坐标系下探测器初始位置向量
    simMars->simIntv->Set_InitialValue(initV);  // 设置惯性坐标系下探测器初始速度向量
    sim1.Set_EnableStore(false);
    sim1.Initialize();  // 仿真器初始化
    // sim1.Simulate();  // 一次性跑完仿真
    // sim1.Plot();  // 波形显示
    cout << "Calculating trajectory......" << endl;
    list<Vector3> Points;  // 存储轨迹点
    Mat point(3, 1);  // 记录轨迹点
    int pointsnum = 1000;
    int process;
    Mat hmat;  // 高度
    double h;
    double vabs = -1;
    int cnt=10;
    for (int n=0; n<pointsnum; n++) {
        sim1.Simulate_OneStep();  // 仿真一个步长
        point = simMars->simIntr->Get_OutValue();  // 记录轨迹点
        Points.push_back((Vector3){  // 存储轨迹点
            float(point.at(0,0)), 
            float(point.at(1,0)), 
            float(point.at(2,0)),
        });
        hmat = simMars->simIntr->Get_OutValue();
        if (cnt>=10) {
            h = Vector3d(hmat).norm2() - MARS_R;
            process = int(n*100.0/pointsnum+0.5);
            cout << "height:" << h*1e3 << ";  process:";
            cout << process << "\%        \r";
            cnt=0;
        }
        cnt++;
        if (h<HMIN) break;
        if (h>H_ATMOS && vabs<0)
        vabs = Vector3d(simMars->simIntv->Get_OutValue()).norm2();
    }
    cout << "\nTrajectory calculating finished." << endl;
    cout << simMars->simIntr->Get_OutValue() << endl;
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
                DrawGrid(120, 5);
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
