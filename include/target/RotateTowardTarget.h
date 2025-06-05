//
// Created by fogoz on 26/05/2025.
//

#ifndef PAMITEENSY_ROTATETOWARDTARGET_H
#define PAMITEENSY_ROTATETOWARDTARGET_H
#include "AngleTarget.h"
#include "../robot/BaseRobot.h"

#define MAKE_ROTATE_TOWARD_TARGET(pos, ramp_data, ramp_type) std::make_shared<RotateTowardTarget<ramp_type>>(robot, pos, ramp_data)

#define DEFAULT_MAKE_ROTATE_TOWARD_TARGET(pos, ramp_data) MAKE_ROTATE_TOWARD_TARGET(pos, ramp_data, CalculatedQuadramp)

#define COMPLETE_ROTATE_TOWARD_TARGET(pos, ramp_data) robot->addTarget(DEFAULT_MAKE_ROTATE_TOWARD_TARGET(pos, ramp_data))

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
