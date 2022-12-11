#include "spacecraft.hpp"

Spacecraft::Spacecraft(Simulator *sim, std::string name) {
    _name = name;
    simfLD = new MFcnMISO(sim, BusSize(3, 1), "simfL");  // 多入单出函数fLD,输出气动力向量f_{LD}
    simfA = new MFcnMISO(sim, BusSize(3, 1), "simfA");  // 多入单出函数fA,输出加速度向量
    simIntr = new MStateSpace(sim, BusSize(3, 1), true, "simIntr");  // 积分器输出位置向量r
    simIntv = new MStateSpace(sim, BusSize(3, 1), true, "simIntv");  // 积分器输出速度向量v
    simgain = new MGain(sim, Mat(vecdble{1}), true, "simgain");  // 子模块的输入端口

    sim->connectM(simfA, simIntv);  // 连接函数fA与速度向量
    sim->connectM(simIntv, simIntr);  // 连接速度向量与位置向量
    sim->connectM(simIntr, simfLD);  // 函数fL的输入参数r
    sim->connectM(simIntv, simfLD);  // 函数fL的输入参数v
    sim->connectM(simgain, simfLD);  // 函数fL的输入参数sigma
    simfLD->Set_Function([](Mat *u){  // 函数fL
        Vector3d r = u[0];
        Vector3d v = u[1];
        double sigma = u[2].at(0, 0);
        double h = r.norm2() - MARS_R;  // 距火星表面高度
        double rho = (h<H_ATMOS) ? rho0*exp(-h/hs) : 0;  // 当前高度下的大气密度
        double Vnorm = v.norm2();  // 速度大小
        double fnorm = rho*Vnorm*Vnorm*Sref*CD;  // 阻力大小
        Vector3d D = -fnorm/Vnorm*v;  // 阻力向量
        fnorm *= LD;  // 升力大小
        Vector3d n1 = r.Normalvector();  // 组成升力的正交单位向量
        Vector3d n2 = (r & v).Normalvector();  // 组成升力的正交单位向量
        Vector3d L = n1*fnorm*cos(sigma) + n2*fnorm*sin(sigma);  // 升力向量
        return Mat(L+D);  // 气动力向量
    });
    sim->connectM(simIntr, simfA);  // 函数fA的输入参数r
    sim->connectM(simfLD, simfA);  // 函数fA的输入参数fLD
    simfA->Set_Function([](Mat *u){  // 函数fA
        Vector3d r = u[0];
        Vector3d fLD = u[1];
        double k = r.norm2();
        k = -MARS_MU / (k*k*k);
        return Mat(k*r + fLD/USV_M);
    });
};

PMatModule Spacecraft::Get_InputBus(int n) const {
    if (n==0) return simgain;
    return nullptr;
};
PMatModule Spacecraft::Get_OutputBus(int n) const {
    if (n==0) return simIntr;
    return nullptr;
};
