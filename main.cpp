#include <cmath>
#include <iostream>
#include <list>
#include "spacecraft.hpp"
#include "identifier.hpp"
#include "camera.h"

// #define DEBUG
#define FULL_SCREEN

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

/**********************
计算简化模型`usv`在常值倾侧角`sigma`输入下的远拱点高度
**********************/
double Calc_Apoapsis(Spacecraft& usv) {
    usv.sim1.Simulation_Reset();
    Vector3d vecr, vecv;  // 位置和速度向量
    double height;  // 高度
    while (1) {
        usv.sim1.Simulate_OneStep();
        vecr = Vector3d(usv.mssIntr->Get_OutValue());
        height = vecr.norm2() - MARS_R;
        if (height > MARS_ATMOS_H) break;  // 飞出大气层
        if (height < USV_HMIN) {  // 高度过低
            cout << "Simplified simulation: Height too low!" << endl; exit(1);
        }
    }
    vecv = Vector3d(usv.mssIntv->Get_OutValue()) / SPFY_RATE;  // 速度向量
    double rnorminv = 1/(height + MARS_R);  // 位置向量长度的倒数
    double energy = vecv*vecv*0.5 - MARS_MU*rnorminv;  // 轨道能量
    if (energy > 0)
        return infinity;
    double a = -0.5*MARS_MU/energy;  // 半长轴
    Vector3d ham = vecr & vecv;  // 角动量(angular momentum)
    double e = (vecv & ham /MARS_MU - vecr*rnorminv).norm2();  // 偏心率
    return a*(1+e);
}

int main(void) {
/**********************
仿真部分
**********************/
    Spacecraft usvStd;  // 飞行器参考简化模型(unmanned space vehicle, standard)
    Spacecraft usvDta;  // 飞行器增量简化模型(unmanned space vehicle, delta)
    Spacecraft usvReal;  // 飞行器精确模型
    double rstd, rdta;  // 远拱点高度的参考值和增量值
    double sigma=0.2;  // 当前倾侧角
    double Ak;  // 输入变换系数
    double yk;  // 变换后的远拱点误差
    double u;  // 制导控制量
    double h;  // 精确仿真模型中的飞行器高度
    double t = 0;  // 仿真时间
    Vector3d debugr,debugv;
    ParamIdentifier idf;
    Mat matr(vecdble{MARS_R+USV_HINIT, 0, 0});  // 位置向量
    Mat matv(vecdble{-SPFY_USV_V*sin(USV_Gamma), SPFY_USV_V*cos(USV_Gamma), 0});  // 速度向量
    usvStd.marsMu = SPFY_MARS_MU;
    usvDta.marsMu = SPFY_MARS_MU;
    usvReal.marsMu = REAL_MARS_MU;
    usvReal.mssIntr->Set_InitialValue(matr);
    usvReal.mssIntv->Set_InitialValue(Mat(vecdble{-REAL_USV_V*sin(USV_Gamma), REAL_USV_V*cos(USV_Gamma), 0}));
    while (t < 100) {
        /*更新参考模型的输入倾侧角和起始位置，计算远拱点高度*/
        usvStd.cnstSigma->Set_OutValue(sigma);
        usvStd.mssIntr->Set_InitialValue(matr);
        usvStd.mssIntv->Set_InitialValue(matv);
        rstd = Calc_Apoapsis(usvStd);
        /*更新增量模型的输入倾侧角和起始位置，计算远拱点高度*/
        usvDta.cnstSigma->Set_OutValue(sigma+0.1);
        usvDta.mssIntr->Set_InitialValue(matr);
        usvDta.mssIntv->Set_InitialValue(matv);
        rdta = Calc_Apoapsis(usvDta);
        /*计算被控对象输出值，即经过输入变换的远拱点误差*/
        Ak = (rdta - rstd) / (cos(sigma+0.1) - cos(sigma));
        if (Ak == 0) {  // 制导律失效
            cout << "Guidance error! (1)" << t << endl; exit(1);
        }
        yk = rstd / Ak;
        if (yk > 1) {  // 制导律失效
            cout << "Guidance error! (2)" << t << endl; exit(1);
        }
        yk = acos(yk);
        /*将远拱点误差送入制导律，计算控制量*/
        sigma = idf.Update(yk);
        /*以当前控制量仿真1秒，即一个离散周期*/
        usvReal.cnstSigma->Set_OutValue(sigma);
        debugv=Vector3d(usvReal.mssIntv->Get_OutValue());
        for (int i = 0; i < 100; i++){
            usvReal.sim1.Simulate_OneStep();
            debugv=Vector3d(usvReal.mssIntv->Get_OutValue());
        }
        t += 0.1;
        /*一个离散周期后，保存位置和速度向量供下一个周期使用*/
        matr = usvReal.mssIntr->Get_OutValue();
        matv = usvReal.mssIntv->Get_OutValue();
        debugv=Vector3d(matv);
        matv = matv * SPFY_RATE / REAL_RATE;
        debugv=Vector3d(matv);
        h = Vector3d(matr).norm2() - MARS_R;
        cout << h << "\r";
        if (h < USV_HMIN) {  // 高度过低
            cout << "Real simulation: Height too low!" << t << endl; exit(1);
        }
    }
    return 0;
}
