#include "orbitalsim.hpp"
#include "utils.h"
#include "zhnmat.hpp"
using namespace zhnmat;
#define PI 3.14159265358979323846f
#define DEG2RAD (PI/180.0)

constexpr double MU_MARS = 42808;  // 火星μ值
constexpr double Rmars = 6804.9/2;  //火星半径

/*********************
轨道六要素转位置和速度
抛物线不适用
输入:6x1矩阵，[a;e;i;Ω;ω;f]
输出:6x1矩阵，[rx;ry;rz;vx;vy;vz]
**********************/
Mat OrbitalElementToPositionVelocity(Mat orbitalElement)
{
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

/*
轨道初值
输入: v-初速度大小. Gamma-进入航迹角(rad). i-轨道倾角(rad)
输出轨道参数。前六个量是轨道位置速度向量,后面四个量依次是a,e,f,hp
*/
Mat IniOrbit(double v, double Gamma, double i)
{
    Vector3d StartLocation, StartVelocity;
    double r = Rmars + 125;  // 距地面125公里处开始算起
    double vinf = sqrt(v*v - 2*MU_MARS/r);  // 无穷远处的速度
    double a = -MU_MARS / (vinf*vinf);  // 双曲线半长轴a
    double hh2 = r*v*cos(Gamma); hh2 *= hh2;  // 角动量的平方
    double e=sqrt(1-hh2/a/MU_MARS);  // 双曲线轨道偏心率e
    double f=acos((hh2/MU_MARS/r-1)/e);  // 真近点角，设进入位置的真近点角为正
    double rmin=a*(1-e);  // 近拱点与火心距离
    double hp=rmin-Rmars;  // 近拱点距地面高度
      // 开始计算位置速度向量
    // xx=a*(1-e^2)/(1+e*cos(f))*[cos(f);sin(f)*cos(i);sin(f)*sin(i)];  // 计算结果与下面式子等价
    Mat xx = r * Mat(3, 1, vecdble{cos(f), sin(f), 0});
    Mat P(3, 1, vecdble{1, 0, 0});
    Q=[0;cos(0);sin(0)];
    dxx=-sqrt(u/a/(1-e^2))*(-sin(f)*P+(e+cos(f))*Q);
    Cy=[cos(i) 0 -sin(i)
        0  1  0
        sin(i) 0 cos(i)];  // 轨道倾角，绕y轴转i
    xx=Cy*xx;
    dxx=Cy*dxx;
    y=[xx;dxx;a;e;f;hp];
}

int main(void) {
    /*参数设置*/
    double Tm = 88642;  // 火星轨道周期
    double wm = 2*PI/Tm; // 火星自转角速度
    double us = 132706538113.88972582103043085267;  // 太阳μ值
    double Fp = 490;  // 发动机推力
    double m = 2500;  // 飞行器质量
    double v=5.8;  // 进入初速度(km/s)
    double Gamma=-10*DEG2RAD;  // 进入航迹角(rad)
    double i=30*DEG2RAD;  //轨道倾角(rad)
    Mat OrbitInfo=IniOrbit(v, Gamma, i);  // 轨道初值
    Mat yyi=OrbitInfo(1:6);  // 初始位置速度向量
    double ri=sqrt(yyi(1)^2+yyi(2)^2+yyi(3)^2);  // 初始位置向量
    double vi=sqrt(yyi(4)^2+yyi(5)^2+yyi(6)^2);  // 初始速度向量
    double rmin=ri;
    double H=ri-Rm;
    double hh0=cross([yyi(1);yyi(2);yyi(3)],[yyi(4);yyi(5);yyi(6)]);  // 轨道初始角动量,h=r×v
    double hh0=hh0/sqrt(hh0'*hh0);  // 初始角动量方向,即轨道平面方向

    /*初始化场景*/
    Camera camera;
    SetWindowMonitor(1);
	SetConfigFlags(FLAG_MSAA_4X_HINT);
	SetConfigFlags(FLAG_FULLSCREEN_MODE);
    SetTargetFPS(60);
    InitGraph(0, 0, "Mars Areocapture");
	Init_Camera(&camera);
    Init_Skybox();

    /*加载卫星*/
    Orbital_Object satellite;
    satellite.Set_initRV(Vector3{15, 0, 0}, Vector3{0, 20, 0});
    satellite.Load_Model("resources/satellite.obj", "resources/satellite.png");
    satellite.Set_Scale(0.01);

    /*加载火星和太阳*/
    Model modelMars = LoadModel("resources/sphere.obj");
    Texture2D imgMars = LoadTexture("resources/Mars.png");
    modelMars.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = imgMars;
    Model modelSun = LoadModel("resources/sphere.obj");
    Texture2D imgSun = LoadTexture("resources/Sun.png");
    modelSun.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = imgSun;

    /*添加全局变量*/
    int cnt = 30;
    int showfps=0, showcpu=0;
    float showspeed;

    /*场景循环*/
    while (!WindowShouldClose()) {
        satellite.Simulate(10);
		Update_Camera(&camera);
        BeginDrawing();
            ClearBackground(RAYWHITE);
            BeginMode3D(camera);
                Update_Skybox();
                DrawModel(modelMars, (Vector3){ 0.0f, 0.0f, 0.0f }, 10.0f, WHITE);
                DrawModel(modelSun, (Vector3){ 100.0f, 0.0f, 0.0f }, 10.0f, WHITE);
                satellite.Update_Model();
            EndMode3D();
            cnt++;
            if (cnt>=30) {
                cnt = 0;
                showfps = GetFPS();
                showcpu = GetCPUusage();
                showspeed = GetCameraSpeed();
            }
            DrawText(TextFormat("FPS: %2i", showfps), 0, 0, 20, LIME);
            DrawText(TextFormat("CPU: %2i %%", showcpu), 0, 30, 20, LIME);
            DrawText(TextFormat("Camera Speed: %.4f", showspeed), 0, 60, 20, LIME);
        EndDrawing();
    }
    CloseGraph();
    return 0;
}
