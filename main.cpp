#include <cmath>
#include <iostream>
#include <list>
#include "spacecraft.hpp"
#include "identifier.hpp"

// #define DEBUG
#define FULL_SCREEN

#ifndef DEBUG
#include "camera.h"
/*类型转换*/
Vector3 MatToVector3(Mat m) {
    return (Vector3) {float(m.at(0,0)), float(m.at(1,0)), float(m.at(2,0))};
}
/*画轨迹*/
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
#endif

/*由位置速度向量计算远拱点高度*/
double RV_to_Apoapsis(Mat r, Mat v) {
    Vector3d vecr = Vector3d(r);  // 位置向量
    Vector3d vecv = Vector3d(v);  // 速度向量
    double rnorminv = 1/(vecr.norm2());  // 位置向量长度的倒数
    double energy = vecv*vecv*0.5 - MARS_MU*rnorminv;  // 轨道能量
    if (energy >= 0) {  // 没有入轨
        cout << "Hyperbolic orbit." << endl;
        return infinity;
    }
    double a = -0.5*MARS_MU / energy;  // 半长轴
    Vector3d ham = vecr & vecv;  // 角动量(angular momentum)
    double e = ((vecv & ham) / MARS_MU - vecr*rnorminv).norm2();  // 偏心率
    return a*(1+e);
}

/**********************
计算简化模型`usv`在常值倾侧角`sigma`输入下的远拱点高度
**********************/
double Calc_Apoapsis(Spacecraft& usv) {
    usv.sim1.Simulation_Reset();
    double height;  // 高度
    Mat r, v;  // 位置向量
    while (1) {
        usv.sim1.Simulate_OneStep();
        r = usv.mssIntr->Get_OutValue();
        height = Vector3d(r).norm2() - MARS_R;
        if (height > MARS_ATMOS_H) break;  // 飞出大气层
        if (height < USV_HMIN) {  // 高度过低
            cout << "Simplified simulation: Height too low!" << endl; exit(1);
        }
    }
    return RV_to_Apoapsis(r, usv.mssIntv->Get_OutValue()/SPFY_RATE);
}

int main(void) {
/**********************
仿真部分
**********************/
    Spacecraft usvStd;  // 飞行器参考简化模型(unmanned space vehicle, standard)
    Spacecraft usvDta;  // 飞行器增量简化模型(unmanned space vehicle, delta)
    Spacecraft usvReal;  // 飞行器精确模型
    list<Vector3> points;  // 存储轨迹点
    Mat point(3, 1);  // 记录轨迹点
    double rstd, rdta;  // 远拱点高度的参考值和增量值
    double sigma=0.2;  // 当前倾侧角
    signed char usign = 1;  // 倾侧角正负
    double Ak;  // 输入变换系数
    double yk;  // 变换后的远拱点误差
    double h;  // 精确仿真模型中的飞行器高度
    double t = 0;  // 仿真时间
    ParamIdentifier idf;  // 制导律
    Mat matr(vecdble{MARS_R+USV_HINIT, 0, 0});  // 位置向量
    Mat matv(vecdble{-SPFY_USV_V*sin(USV_Gamma), SPFY_USV_V*cos(USV_Gamma), 0});  // 速度向量
    usvStd.marsMu = SPFY_MARS_MU;
    usvDta.marsMu = SPFY_MARS_MU;
    usvReal.marsMu = REAL_MARS_MU;
    usvReal.mssIntr->Set_InitialValue(matr);
    usvReal.mssIntv->Set_InitialValue(Mat(vecdble{-REAL_USV_V*sin(USV_Gamma), REAL_USV_V*cos(USV_Gamma), 0}));
    cout << "Calculating trajectory inside atmosphere......" << endl;
    while (t < 100) {
        /*更新参考模型的输入倾侧角和起始位置，计算远拱点高度*/
        usvStd.cnstSigma->Set_OutValue(sigma);
        usvStd.mssIntr->Set_InitialValue(matr);
        usvStd.mssIntv->Set_InitialValue(matv);
        rstd = Calc_Apoapsis(usvStd);
        /*更新增量模型的输入倾侧角和起始位置，计算远拱点高度*/
        usvDta.cnstSigma->Set_OutValue(sigma + Dsigma);
        usvDta.mssIntr->Set_InitialValue(matr);
        usvDta.mssIntv->Set_InitialValue(matv);
        rdta = Calc_Apoapsis(usvDta);
        /*计算被控对象输出值，即经过输入变换的远拱点误差*/
        // if (sigma + Dsigma < 0) {  // 制导律失效
        //     cout << "Guidance error!(2) " << t << endl; exit(1);
        // }
        Ak = (rdta - rstd) / (cos(sigma + Dsigma) - cos(sigma));
        // if (Ak == 0) {  // 制导律失效
        //     cout << "Guidance error!(1) " << t << endl; exit(1);
        // }
        yk = (rstd - TARGET_APOAPSIS) / Ak;
        /*将远拱点误差送入制导律，计算控制量*/
        sigma = idf.Update(yk);
        if (matv.at(2, 0) > 2500 * exp(-0.02*t))
            usign = -1;
        else if (matv.at(2, 0) < -2500 * exp(-0.02*t))
            usign = 1;
        sigma = usign * acos(sigma);
        /*以当前控制量仿真1秒，即一个离散周期*/
        usvReal.cnstSigma->Set_OutValue(sigma);
        for (int i = 0; i < 100; i++)
            usvReal.sim1.Simulate_OneStep();
        point = usvReal.mssIntr->Get_OutValue() * 1e-3;
        points.push_back(MatToVector3(point));
        t += 0.1;
        /*一个离散周期后，保存位置和速度向量供下一个周期使用*/
        matr = usvReal.mssIntr->Get_OutValue();
        matv = usvReal.mssIntv->Get_OutValue();
        matv = matv * SPFY_RATE / REAL_RATE;
        h = Vector3d(matr).norm2() - MARS_R;
        /*调试、打印与判断*/
        cout << t << "s, height: " << h << "        \r";
        // cout << t << ", " << h << ", " << sigma << ", " << matv.at(2, 0) << endl;
        // cout << sigma << matr << matv << endl;
        // cout << idf._theta << endl;
        if (h < USV_HMIN) {  // 高度过低
            cout << "Real simulation: Height too low!" << t << endl; exit(1);
        }
        if (h > MARS_ATMOS_H)  // 飞出大气层
            break;
    }
    t = 0;
    cout << "\nFlew out of the atmosphere." << endl;
    cout << "Target Apoapsis: " << TARGET_APOAPSIS << endl;
    cout << "Calculated Apoapsis: " << RV_to_Apoapsis(matr, matv/SPFY_RATE) << endl;
    cout << "Calculating trajectory outside atmosphere......" << endl;
    // return 0;
    int process;
    double total = 4;
    while (t < total*100) {
        for (int i = 0; i < 1000; i++)
            usvReal.sim1.Simulate_OneStep();
        point = usvReal.mssIntr->Get_OutValue() * 1e-3;
        points.push_back(MatToVector3(point));
        t += 1;
        process = t/total + 0.5;
        cout << process << "\%\r";
    }
    cout << "\nTrajectory calculating finished." << endl;
    // cout << usvReal.mssIntr->Get_OutValue() << endl;
    h = Vector3d(usvReal.mssIntr->Get_OutValue()).norm2();
    cout << "Current distance after " << total*100 << " seconds: " << h << endl;

/**********************
绘图
**********************/
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
                DrawGrid(120, 6);
                DrawSphere((Vector3){0.0f, 0.0f, 0.0f}, 2, ORANGE);
                Draw_Trajectory(points);
            EndMode3D();
            DrawText(TextFormat("%2i FPS", GetFPS()), 0, 0, 20, LIME);
        EndDrawing();
    }
    CloseGraph();
#endif
    return 0;
}
