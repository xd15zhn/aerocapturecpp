#ifndef SPACECRAFT_H
#define SPACECRAFT_H

#include <cmath>
#include "simucpp.hpp"
#include "parameters.hpp"
using namespace simucpp;
using namespace zhnmat;
using namespace std;

/**********************
火星探测器被控对象
**********************/
class Spacecraft {
public:
    Spacecraft();
    ~Spacecraft() {};
    Simulator sim1;  // 添加仿真器(#include "simucpp.hpp")
    MFcnMISO *fcnfLD=nullptr;  // 多入单出函数fLD,输出气动力向量f_{LD}
    MFcnMISO *fcnfA=nullptr;  // 多入单出函数fA,输出加速度向量
    MStateSpace *mssIntr=nullptr;  // 积分器输出位置向量r
    MStateSpace *mssIntv=nullptr;  // 积分器输出速度向量v
    UConstant *cnstSigma=nullptr;  // 常数模块输出倾侧角
    Mux *muxSigma=nullptr;  // 单线转总线
    double marsMu;
};
typedef Spacecraft* PSpacecraft;

#endif // SPACECRAFT_H
