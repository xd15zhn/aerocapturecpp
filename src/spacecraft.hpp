#ifndef SPACECRAFT_H
#define SPACECRAFT_H

#include <cmath>
#include "simucpp.hpp"
using namespace simucpp;
using namespace zhnmat;
using namespace std;

#define PI 3.14159265358979323846f

constexpr double LD = 0.28;  // 升阻比
/*下面的值需要换算到求解器时空坐标下，注释中为换算前的单位*/
constexpr double MARS_MU = 42808e-1;  // 火星μ值
constexpr double MARS_R = 3402.5e-3;  //火星半径(km)
constexpr double USV_V = 5.8e1;  // 进入初速度(km/s)
constexpr double USV_M=2500;  // 飞行器质量(kg)
constexpr double USV_H = 125e-3;  // 飞行器进入点高度
constexpr double USV_Gamma = 0.175;  // 飞行器进入航迹角
constexpr double rho0 = 0.5*14740000e9;  // 参考密度(kg/km^2)
constexpr double hs = 8.8057e-3;  // 比例高度(km)
constexpr double Sref = 17.44e-12;  // 飞行器参考面积(km^2)
constexpr double CD = 1.43;  // 阻力系数
constexpr double H_ATMOS = 150e-3;  // 火星大气厚度
constexpr double HMIN = 10e-3;  // 飞行器最低允许飞行高度

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
};
typedef Spacecraft* PSpacecraft;

#endif // SPACECRAFT_H
