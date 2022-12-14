#ifndef IDENTIFIER_H
#define IDENTIFIER_H

#include "simucpp.hpp"
using namespace simucpp;
using namespace zhnmat;
using namespace std;

#define LIMIT(x, min, max)           (((x)<=(min) ? (min) : ((x)>=(max) ? (max) : (x))))


/**********************
包含特征模型参数辨识的制导律
**********************/
class ParamIdentifier {
public:
    ParamIdentifier() {
        _P = eye(2) * 1e6;  // P矩阵初始值
        _theta = Mat(vecdble{0.5, 0.5});  // 待辨识参数初始值
        _u = 0.2;  // 控制量初始值
        _lambda = 0.5;
        _l1 = 0.2;
        _yk1 = 0;
    };
    double Update(double yk) {
        Mat x(vecdble{_yk1, _u});
        Mat K = 1/(_lambda + (x.T()*_P*x).at(0, 0)) * _P * x;
        _theta += K * (yk - (x.T()*_theta).at(0, 0));
        _yk1 = yk;
        double _alpha1 = LIMIT(_theta.at(0, 0), 0.1, 0.9);
        double _beta0 = LIMIT(_theta.at(1, 0), 0.1, 0.9);
        _theta.set(0, 0, _alpha1);
        _theta.set(1, 0, _beta0);
        double uL = -(_l1 * _alpha1 * yk) / _beta0;
        // _u = LIMIT(_u + uL, 0.26, 2.88);
        _u = LIMIT(_u + uL, -0.97, 0.97);
        return _u;
    }
    Mat _P;  // P矩阵
    Mat _theta;  // 特征模型参数(_alpha1, _beta0)
    double _lambda;  // 遗忘因子
    double _yk1;  // y(k-1)
    double _l1;  // 制导律参数
    double _u;  // 控制量
};

#endif // IDENTIFIER_H
