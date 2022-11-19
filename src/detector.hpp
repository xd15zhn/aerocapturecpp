#ifndef ORBITALSIM_H
#define ORBITALSIM_H

#include <cmath>
#include "simucpp.hpp"
using namespace simucpp;
using namespace zhnmat;
using namespace std;

#define PI 3.14159265358979323846f

constexpr double LD = 0.28;  // 升阻比
/*下面的值需要换算到求解器时空坐标下，注释中为换算前的单位*/
constexpr double MU_MARS = 42808e-3;  // 火星μ值
constexpr double R_MARS = 3402.5e-3;  //火星半径(km)
constexpr double v_USV = 5.8;  // 进入初速度(km/s)
constexpr double rho0 = 0.5*14740000e9;  // 参考密度(kg/km^2)
constexpr double hs = 8.8057e-3;  // 比例高度(km)
constexpr double Sref = 17.44e-12;  // 飞行器参考面积(km^2)
constexpr double CD = 1.43;  // 阻力系数

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
    MFcnMISO *simfLD=nullptr;
    MFcnMISO *simfA=nullptr;
    MStateSpace* simIntr=nullptr;
    MStateSpace* simIntv=nullptr;
    MGain *simgain=nullptr;
};
typedef MarsDetector* PMarsDetector;

#endif // ORBITALSIM_H
