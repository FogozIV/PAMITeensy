//
// Created by fogoz on 08/05/2025.
//
#pragma once

#include "AngleTarget.h"

#include "ramp/BasicQuadRamp.h"
#include "robot/BaseRobot.h"

template<typename T>
void AngleTarget<T>::on_done() {
    robot->setDoneDistance(true);
    robot->setDoneAngular(true);
    robot->setTranslationalTarget(robot->getTranslationalPosition());
    if (ramp_data.endSpeed == 0) {
        robot->getController()->reset();
    }
}
template<typename T>
bool AngleTarget<T>::is_done() {
    return done;
}
template<typename T>
void AngleTarget<T>::init() {
    robot->setRotationalPosition(robot->getCurrentPosition().getAngle());
    robot->setRotationalTarget(robot->getRotationalPosition());
    ramp = std::make_shared<T>(robot, ramp_data, [this]() {
        return (target_angle - robot->getCurrentPosition().getAngle()).warpAngle().toDegrees();
    });
    ramp->start(robot->getRotationalRampSpeed().toDegrees());
    robot->setDoneDistance(true);
    robot->setDoneAngular(false);
}
template<typename T>
void AngleTarget<T>::process() {
    double target = ramp->computeDelta();
    robot->setRotationalRampSpeed(Angle::fromDegrees(ramp->getCurrentSpeed()));
    robot->setRotationalTarget(robot->getRotationalTarget() + Angle::fromDegrees(target));
    if (abs((target_angle - robot->getCurrentPosition().getAngle()).toDegrees()) < 2) {
        done = true;
    }
}
template<typename T>
void AngleTarget<T>::reInitAfterStop() {
    ramp->stop();
}
