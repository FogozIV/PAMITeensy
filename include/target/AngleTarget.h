//
// Created by fogoz on 08/05/2025.
//

#ifndef ANGLETARGET_H
#define ANGLETARGET_H
#include "BaseTarget.h"
#include "ramp/Ramp.h"
#include "utils/Angle.h"


template<typename T>
class AngleTarget : public BaseTarget{
    bool done = false;
    std::shared_ptr<Ramp> ramp;
    RampData ramp_data;
    Angle target_angle;
public:
    explicit AngleTarget(const std::shared_ptr<BaseRobot> &robot, Angle target_angle, RampData rampData)
        : BaseTarget(robot), ramp_data(rampData), target_angle(target_angle) {
    }

    void on_done() override;

    bool is_done() override;

    void init() override;

    void process() override;

    void reInitAfterStop() override;
};

#include "AngleTarget.tpp"



#endif //ANGLETARGET_H
