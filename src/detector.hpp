#ifndef ORBITALSIM_H
#define ORBITALSIM_H

#include <cmath>
#include "simucpp.hpp"
using namespace simucpp;
using namespace zhnmat;
using namespace std;

#define PI 3.14159265358979323846f
#define DEG2RAD (PI/180.0)

constexpr double MU_MARS = 42808;  // 火星μ值
constexpr double R_MARS = 6804.9/2.0;  //火星半径(km)
constexpr double T_MARS = 88642;  // 火星轨道周期
constexpr double W_MARS = 2*PI/T_MARS; // 火星自转角速度
constexpr double MU_SUN = 132706538113.88972582103043085267;  // 太阳μ值
constexpr double Fp = 490;  // 发动机推力
constexpr double M_USV = 2500;  // 飞行器质量
constexpr double v_USV = 5.8;  // 进入初速度(km/s)
constexpr double rho0 = 0.5*1.474e7;  // 参考密度(kg/km^2)
constexpr double hs = 8.8057;  // 比例高度(km)
constexpr double Sref = 17.44e-6;  // 飞行器参考面积(km^2)
constexpr double CD = 1.43;  // 阻力系数
constexpr double CL = 0.4;  // 升力系数

/**********************
火星探测器被控对象
**********************/
class MarsDetector: public PackModule {
public:
    MarsDetector(Simulator *sim, std::string name);
    ~MarsDetector() {};
    virtual PMatModule Get_InputBus(int n) const;
    virtual PMatModule Get_OutputBus(int n) const;
    // Mat Get_Position();
    // Mat Get_Velocity();
    std::string _name;
    MFcnMISO *simf1=nullptr;
    MFcnMISO *simf2=nullptr;
    MFcnMISO *simf3=nullptr;
    MStateSpace* simIntv=nullptr;
    MStateSpace* simIntr=nullptr;
    MGain *simgain=nullptr;
};
typedef MarsDetector* PMarsDetector;

#endif // ORBITALSIM_H
