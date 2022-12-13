#include "spacecraft.hpp"

Spacecraft::Spacecraft() {
    fcnfLD = new MFcnMISO(&sim1, BusSize(3, 1), "fcnfLD");  // 多入单出函数fLD,输出气动力向量f_{LD}
    fcnfA = new MFcnMISO(&sim1, BusSize(3, 1), "fcnfA");  // 多入单出函数fA,输出加速度向量
    mssIntr = new MStateSpace(&sim1, BusSize(3, 1), true, "mssIntr");  // 积分器输出位置向量r
    mssIntv = new MStateSpace(&sim1, BusSize(3, 1), true, "mssIntv");  // 积分器输出速度向量v
    cnstSigma = new UConstant(&sim1, "cnstSigma");  // 往仿真器中添加输入模块
    muxSigma = new Mux(&sim1, BusSize(1, 1), "muxSigma");  // 往仿真器中添加总线复用模块，将倾侧角输入的标量信号转换为矩阵信号

    sim1.connectU(cnstSigma, muxSigma, BusSize(0, 0));  // 连接倾侧角与总线复用
    sim1.connectM(fcnfA, mssIntv);  // 连接函数fA与速度向量
    sim1.connectM(mssIntv, mssIntr);  // 连接速度向量与位置向量
    sim1.connectM(mssIntr, fcnfLD);  // 函数fL的输入参数r
    sim1.connectM(mssIntv, fcnfLD);  // 函数fL的输入参数v
    sim1.connectM(muxSigma, fcnfLD);  // 函数fL的输入参数sigma
    fcnfLD->Set_Function([=](Mat *u){  // 函数fL
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
    sim1.connectM(mssIntr, fcnfA);  // 函数fA的输入参数r
    sim1.connectM(fcnfLD, fcnfA);  // 函数fA的输入参数fLD
    fcnfA->Set_Function([=](Mat *u){  // 函数fA
        Vector3d r = u[0];
        Vector3d fLD = u[1];
        double k = r.norm2();
        k = -MARS_MU / (k*k*k);
        return Mat(k*r + fLD/USV_M);
    });
    sim1.Set_EnableStore(false);  // 单步仿真不需要存储数据
    sim1.Initialize();  // 仿真器初始化
};
