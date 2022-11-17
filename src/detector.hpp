#ifndef ORBITALSIM_H
#define ORBITALSIM_H

#include <cmath>
#include "simucpp.hpp"
using namespace simucpp;
using namespace zhnmat;
using namespace std;

/**********************
火星探测器被控对象
**********************/
class MarsDetector: public PackModule {
public:
    MarsDetector(Simulator *sim, std::string name);
    ~MarsDetector() {};
    virtual PMatModule Get_InputBus(int n) const;
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
