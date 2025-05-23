//
// Created by fogoz on 10/05/2025.
//

#ifndef CURVETARGET_H
#define CURVETARGET_H
#include "BaseTarget.h"
#include "curves/BaseCurve.h"
#include "ramp/Ramp.h"

template<typename Ramp>
class CurveTarget : public BaseTarget{
    bool done = false;
    RampData rampData;
    std::shared_ptr<BaseCurve> curve;
    std::shared_ptr<Ramp> ramp;
    double t;
    double step;
    Position target_pos;
public:
    explicit CurveTarget(const std::shared_ptr<BaseRobot> &robot, std::shared_ptr<BaseCurve> curve, RampData ramp, double step=20.0);

    bool is_done() override;

    void init() override;

    void on_done() override;

    void process() override;

    void reInitAfterStop() override;
};




#include "CurveTarget.tpp"

#endif //CURVETARGET_H
