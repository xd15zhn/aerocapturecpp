#include "detector.hpp"

/**********************
一个卫星
**********************/
MarsDetector::MarsDetector(Simulator *sim, std::string name) {
    _name = name;
    simf1 = new MFcnMISO(sim, BusSize(3, 1), "simf1");  // 多入单出函数f1,输出向量L
    simf2 = new MFcnMISO(sim, BusSize(3, 1), "simf2");  // 多入单出函数f2,输出向量D
    simf3 = new MFcnMISO(sim, BusSize(3, 1), "simf3");  // 多入单出函数f3,输出加速度向量
    simIntr = new MStateSpace(sim, BusSize(3, 1), true, "simIntr");  // 积分器输出位置向量r
    simIntv = new MStateSpace(sim, BusSize(3, 1), true, "simIntv");  // 积分器输出速度向量v
    simgain = new MGain(sim, Mat(vecdble{1}), true, "simgain");

    sim->connectM(simf3, simIntv);
    sim->connectM(simIntv, simIntr);
    sim->connectM(simIntr, simf1);  // 函数f1的输入参数r
    sim->connectM(simIntv, simf1);  // 函数f1的输入参数v
    simf1->Set_Function([](Mat *u){  // 函数f1
        Vector3d r = u[0];
        Vector3d v = u[1];
        double h = r.norm2() - R_MARS;
        double rho = rho0 * exp(-h/hs);
        double Vnorm = v.norm2();
        double Dnorm = rho*Vnorm*Vnorm*Sref*CD;
        Dnorm = -Dnorm/Vnorm;
        return Mat(Dnorm*v);
    });
    sim->connectM(simIntr, simf2);  // 函数f2的输入参数r
    sim->connectM(simIntv, simf2);  // 函数f2的输入参数v
    sim->connectM(simgain, simf2);  // 函数f2的输入参数sigma
    simf2->Set_Function([](Mat *u){  // 函数f2
        Vector3d r = u[0];
        Vector3d v = u[1];
        double sigma = u[2].at(0, 0);
        double h = r.norm2() - R_MARS;
        double rho = rho0 * exp(-h/hs);
        double Vnorm = v.norm2();
        double Lnorm = rho*Vnorm*Vnorm*Sref*CL;
        Vector3d n2 = (r & v).Normalize();
        Vector3d n1 = r.Normalvector();
        return Mat(n1*Lnorm*cos(sigma) + n2*Lnorm*sin(sigma));
    });
    sim->connectM(simIntr, simf3);  // 函数f3的输入参数r
    sim->connectM(simf2, simf3);  // 函数f3的输入参数L
    sim->connectM(simf1, simf3);  // 函数f3的输入参数D
    simf3->Set_Function([](Mat *u){  // 函数f3
        Vector3d r = u[0];
        Vector3d L = u[1];
        Vector3d D = u[2];
        double k = r.norm2();
        k = -MU_MARS / (k*k*k);
        return Mat(k*r + L + D);
    });
};

PMatModule MarsDetector::Get_InputBus(int n) const {
    if (n==0) return simgain;
    return nullptr;
};
PMatModule MarsDetector::Get_OutputBus(int n) const {
    if (n==0) return simIntr;
    return nullptr;
};
