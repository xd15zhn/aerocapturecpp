#include "detector.hpp"

/**********************
一个卫星
**********************/
MarsDetector::MarsDetector(Simulator *sim, std::string name) {
    _name = name;
    simfh = new MFcnMISO(sim, BusSize(1, 1), "simfh");  // 多入单出函数fh,输出探测器高度h
    simfD = new MFcnMISO(sim, BusSize(3, 1), "simfD");  // 多入单出函数fD,输出向量D
    simfL = new MFcnMISO(sim, BusSize(3, 1), "simfL");  // 多入单出函数fL,输出向量L
    simfA = new MFcnMISO(sim, BusSize(3, 1), "simfA");  // 多入单出函数fA,输出加速度向量
    simIntr = new MStateSpace(sim, BusSize(3, 1), true, "simIntr");  // 积分器输出位置向量r
    simIntv = new MStateSpace(sim, BusSize(3, 1), true, "simIntv");  // 积分器输出速度向量v
    simgain = new MGain(sim, Mat(vecdble{1}), true, "simgain");  // 子模块的输入端口

    sim->connectM(simfA, simIntv);  // 连接函数fA与速度向量
    sim->connectM(simIntv, simIntr);  // 连接速度向量与位置向量
    sim->connectM(simIntr, simfh);  // 函数fh的输入参数r
    simfh->Set_Function([](Mat *u){  // 函数fh
        Vector3d r = u[0];
        double h = r.norm2() - R_MARS;
        return Mat(vecdble{h});
    });
    sim->connectM(simfh, simfD);  // 函数fD的输入参数h
    sim->connectM(simIntv, simfD);  // 函数fD的输入参数v
    simfD->Set_Function([](Mat *u){  // 函数fD
        double h = u[0].at(0, 0);
        Vector3d v = u[1];
        double rho = rho0 * exp(-h/hs);
        double Vnorm = v.norm2();
        double Dnorm = rho*Vnorm*Vnorm*Sref*CD;
        Dnorm = -Dnorm/Vnorm;
        return Mat(Dnorm*v);
    });
    sim->connectM(simIntr, simfL);  // 函数fL的输入参数r
    sim->connectM(simIntv, simfL);  // 函数fL的输入参数v
    sim->connectM(simgain, simfL);  // 函数fL的输入参数sigma
    simfL->Set_Function([](Mat *u){  // 函数fL
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
    sim->connectM(simIntr, simfA);  // 函数fA的输入参数r
    sim->connectM(simfL, simfA);  // 函数fA的输入参数L
    sim->connectM(simfD, simfA);  // 函数fA的输入参数D
    simfA->Set_Function([](Mat *u){  // 函数fA
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
