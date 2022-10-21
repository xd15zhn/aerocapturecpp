#include <cmath>
#include "simucpp.hpp"
using namespace simucpp;
using namespace zhnmat;
using namespace std;
#define PI 3.14159265358979323846f
#define DEG2RAD (PI/180.0)

constexpr double MU_MARS = 42808;  // 火星μ值
constexpr double R_MARS = 6804.9/2;  //火星半径
constexpr double T_MARS = 88642;  // 火星轨道周期
constexpr double W_MARS = 2*PI/T_MARS; // 火星自转角速度
constexpr double MU_SUN = 132706538113.88972582103043085267;  // 太阳μ值

/*********************
轨道六要素转位置速度
抛物线不适用
输入:6x1矩阵，[a;e;i;Ω;ω;f]
输出:6x1矩阵，[rx;ry;rz;vx;vy;vz]
**********************/
Mat OrbitalElementToPositionVelocity(Mat orbitalElement) {
    double a = orbitalElement.at(0, 0);  // a-半长轴
    double e = orbitalElement.at(1, 0);  // e-偏心率
    double i = orbitalElement.at(2, 0);  // i-轨道倾角
    double n = orbitalElement.at(3, 0);  // Ω-升交点赤经
    double w = orbitalElement.at(4, 0);  // ω-近地点幅角
    double f = orbitalElement.at(5, 0);  // f-真近点角
    double mu = MU_MARS;
    double r = a*(1-e*e)/(1+e*cos(f));
    if (e > 1) a = -a;
    double k = sqrt(mu/a*(1-e*e));
    Mat rvec(3, 1, vecdble{r*cos(f), r*sin(f), 0});
    Mat vvec(3, 1, vecdble{-k*sin(f), k*(e+cos(f)), 0});
    Mat RT(3, 3, vecdble{
        cos(n)*cos(w)-sin(n)*cos(i)*sin(w), -cos(n)*sin(w)-sin(n)*cos(i)*cos(w), sin(n)*sin(i),
        sin(n)*cos(w)+cos(n)*cos(i)*sin(w), -sin(n)*sin(w)+cos(n)*cos(i)*cos(w), -cos(n)*sin(i),
        sin(i)*sin(w),                      sin(i)*cos(w),                       cos(i)
    });
    rvec = RT*rvec; vvec = RT*vvec;
    return Mat(6, 1, vecdble{rvec.at(0, 0), rvec.at(1, 0), rvec.at(2, 0), rvec.at(0, 0), vvec.at(1, 0), vvec.at(2, 0)});
}

/*********************
位置速度转轨道六要素
输入:6x1矩阵，[a;e;i;Ω;ω;f]
输出:6x1矩阵，[rx;ry;rz;vx;vy;vz]
**********************/
Mat OrbitalElementToPositionVelocity(Mat positionVelocity) {
    Vector3d vecr(positionVelocity.at(0, 0), positionVelocity.at(1, 0), positionVelocity.at(2, 0));
    Vector3d vecv(positionVelocity.at(3, 0), positionVelocity.at(4, 0), positionVelocity.at(5, 0));
    double mu = MU_MARS;
    double energy = 0.5*vecv*vecv - mu/vecr.norm2();
    Mat ans(6, 1);
    ans.set(0, 0, -0.5*mu/energy);  // a
    Vector3d momentum = vecr & vecv;
    Vector3d eccentricity = vecv & momentum / mu - vecr / vecr.norm2();
    ans.set(1, 0, eccentricity.norm2());  // e
    ans.set(2, 0, acos(momentum._x/momentum.norm2()));  // i
    return ans;
}

int main(void) {
    double Fp = 490;  // 发动机推力
    double m = 2500;  // 飞行器质量
    double v=5.8;  // 进入初速度(km/s)
}
