//
// Created by fogoz on 26/05/2025.
//

#ifndef PAMITEENSY_ROTATETOWARDTARGET_H
#define PAMITEENSY_ROTATETOWARDTARGET_H
#include "AngleTarget.h"
#include "../robot/BaseRobot.h"

template<typename Ramp>
class RotateTowardTarget : public AngleTarget<Ramp>{
    Position target;
public:
    RotateTowardTarget(std::shared_ptr<BaseRobot> robot, Position target, RampData ramp): AngleTarget<Ramp>(robot, AngleConstants::ZERO, ramp){
        this->target = std::move(target);
    };

    void init() override {
        this->target_angle = (target - this->robot->getCurrentPosition()).getVectorAngle();
        AngleTarget<Ramp>::init();
    }

};

#endif //PAMITEENSY_ROTATETOWARDTARGET_H
