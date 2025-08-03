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
    //robot->getController()->reset(0);
    if (ramp_data.endSpeed == 0) {
        //robot->setTranslationalRampSpeed(0);
        //robot->getController()->reset();
    }
}
template<typename T>
bool AngleTarget<T>::is_done() {
    return done;
}
template<typename T>
void AngleTarget<T>::init() {
    /*
    if (robot->getRotationalRampSpeed().toDegrees() == 0.0) {
        robot->setRotationalPosition(robot->getCurrentPosition().getAngle());
        robot->setRotationalTarget(robot->getRotationalPosition());
    }*/
    ramp = std::make_shared<T>(robot, ramp_data, [this]() {
        return (target_angle - robot->getCurrentPosition().getAngle()).warpAngle().toDegrees();
    });
    ramp->start(robot->getRotationalRampSpeed().toDegrees());
    //robot->setRotationalTarget(robot->getRotationalPosition());
    robot->setDoneDistance(true);
    robot->setDoneAngular(false);
}
template<typename T>
void AngleTarget<T>::process() {
    double target = ramp->computeDelta();
    //streamSplitter.println(target);
    robot->setRotationalRampSpeed(Angle::fromDegrees(ramp->getCurrentSpeed()));
    robot->setRotationalTarget(robot->getRotationalTarget() + Angle::fromDegrees(target));
    robot->setPositionTarget((robot->getPositionTarget() + Position(0,0, Angle::fromRadians(target))).warpAngle());
    if (abs((target_angle - robot->getCurrentPosition().getAngle()).warpAngle().toDegrees()) < 2) {
        count++;
        if (count > 100) {
            robot->setRotationalTarget(robot->getRotationalTarget() - Angle::fromDegrees(target));
            robot->setPositionTarget((robot->getPositionTarget() - Position(0,0, Angle::fromRadians(target))).warpAngle());
            done = true;
        }
    }else {
        count = 0;
    }
    if(target == 0.0 && abs((robot->getRotationalTarget() - robot->getRotationalPosition()).warpAngle().toDegrees()) < 2){
        reached_count++;
        if (reached_count > 20) {
            done = true;
        }
    }else {
        reached_count=  0;
    }
    BaseTarget::process();
}
template<typename T>
void AngleTarget<T>::reInitAfterStop() {
    ramp->stop();
}
