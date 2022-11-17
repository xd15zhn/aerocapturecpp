#include "detector.hpp"

#define PI 3.14159265358979323846f
#define DEG2RAD (PI/180.0)

constexpr double MU_MARS = 42808;  // 火星μ值
constexpr double R_MARS = 6804.9/2;  //火星半径
constexpr double T_MARS = 88642;  // 火星轨道周期
constexpr double W_MARS = 2*PI/T_MARS; // 火星自转角速度
constexpr double MU_SUN = 132706538113.88972582103043085267;  // 太阳μ值
constexpr double Fp = 490;  // 发动机推力
constexpr double m = 2500;  // 飞行器质量
constexpr double v = 5.8;  // 进入初速度(km/s)

/**********************
一个卫星
**********************/
MarsDetector::MarsDetector(Simulator *sim, std::string name) {
    _name = name;
    simf1 = new MFcnMISO(sim, BusSize(3, 1), "simf1");  // 多入单出函数f1,输出向量L
    simf2 = new MFcnMISO(sim, BusSize(3, 1), "simf2");  // 多入单出函数f2,输出向量D
    simf3 = new MFcnMISO(sim, BusSize(3, 1), "simf3");  // 多入单出函数f3,输出加速度向量
    simIntv = new MStateSpace(sim, BusSize(3, 1), true, "simIntv");  // 积分器输出速度向量v
    simIntr = new MStateSpace(sim, BusSize(3, 1), true, "simIntr");  // 积分器输出位置向量r

    sim->connectM(simf3, simIntv);
    sim->connectM(simIntv, simIntr);
    sim->connectM(simIntr, simf1);  // 函数f1的输入参数r
    sim->connectM(simIntv, simf1);  // 函数f1的输入参数v
    simf1->Set_Function([](Mat *u){  // 函数f3
        Vector3d r; r = u[0];
        Vector3d v; v = u[1];
        Mat ans; ans = ansvec;
        return ans;
    });
    sim->connectM(simIntv, simf2);
    sim->connectM(simIntr, simf2);
    sim->connectM(simgain, simf2);
    sim->connectM(simIntr, simf3);  // 函数f3的输入参数r
    sim->connectM(simf2, simf3);  // 函数f3的输入参数L
    sim->connectM(simf1, simf3);  // 函数f3的输入参数D
    simf3->Set_Function([](Mat *u){  // 函数f3
        Vector3d r; r = u[0];
        Vector3d L; L = u[1];
        Vector3d D; D = u[2];
        double k = MU_MARS / r.norm2();
        Vector3d ansvec = k*r + L + D;
        Mat ans; ans = ansvec;
        return ans;
    });
};
PMatModule MarsDetector::Get_InputBus(int n) const {
    if (n==0) return simgain;
    return nullptr;
};
