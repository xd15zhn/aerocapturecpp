#ifndef PARAMETERS_H
#define PARAMETERS_H

// constexpr double USV_CL = 0.4;  // 升力系数
constexpr double USV_CD = 1.43;  // 阻力系数
constexpr double USV_LD = 0.28;  // 升阻比
constexpr double MARS_MU = 42808;  // 火星μ值
constexpr double MARS_R = 3402.5;  //火星半径(km)
constexpr double MARS_ATMOS_H = 150;  // 火星大气厚度
constexpr double USV_HMIN = 10;  // 飞行器最低允许飞行高度
constexpr double USV_HINIT = 125;  // 飞行器进入点高度
constexpr double USV_V = 5.8e4;  // 进入初速度(km/s)
constexpr double USV_M=2500;  // 飞行器质量(kg)
constexpr double USV_Gamma = 0.175;  // 飞行器进入航迹角
constexpr double USV_SREF = 17.44e-6;  // 飞行器参考面积(km^2)
constexpr double MARS_rho0 = 0.5*14.74e6;  // 参考密度(kg/km^2)
constexpr double MARS_hs = 8.8057;  // 比例高度(km)
constexpr double SPFY_RATE = 1e4;  // 简化模型的时间加快比例
constexpr double REAL_RATE = 10;  // 精确模型的时间加快比例
/*下面的值对应简化模型中的换算参数*/
constexpr double SPFY_MARS_MU = MARS_MU*SPFY_RATE*SPFY_RATE;
constexpr double SPFY_USV_V = 5.8*SPFY_RATE;
/*下面的值对应精确模型中的换算参数*/
constexpr double REAL_MARS_MU = MARS_MU*REAL_RATE*REAL_RATE;
constexpr double REAL_USV_V = 5.8*REAL_RATE;

constexpr double infinity = 1e12;  // 无穷大
constexpr double Dsigma = 0.1;  // 倾侧角增量
constexpr double TARGET_APOAPSIS = MARS_R + 2250;  // 倾侧角增量

#endif // PARAMETERS_H
