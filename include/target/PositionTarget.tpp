//
// Created by fogoz on 07/05/2025.
//
#pragma once

#include "PositionTarget.h"


template<typename T>
PositionTarget<T>::PositionTarget(std::shared_ptr<BaseRobot> baseRobot, Position pos, RampData rampData): BaseTarget(baseRobot),pos(pos), ramp_data(rampData) {
    distanceComputer = [this]() {
        Angle angle = ((this->pos-robot->getCurrentPosition()).getVectorAngle() - robot->getCurrentPosition().getAngle()).warpAngle();
        if (angle > -90_deg && angle < 90_deg) {
            return (robot->getCurrentPosition() - this->pos).getDistance();
        }
        return -(robot->getCurrentPosition() - this->pos).getDistance();
    };
}

template<typename T>
bool PositionTarget<T>::is_done() {
    return done;
}

template<typename T>
void PositionTarget<T>::init() {
    ramp = std::make_shared<T>(robot, ramp_data, this->distanceComputer);
    ramp->start(robot->getTranslationalRampSpeed());
    robot->setDoneAngular(false);
    robot->setDoneDistance(false);
    robot->setRotationalPosition(robot->getCurrentPosition().getAngle());
}

template<typename T>
void PositionTarget<T>::process() {
    double distance_update = ramp->computeDelta();
    robot->setTranslationalRampSpeed(ramp->getCurrentSpeed());
    robot->setTranslationalTarget(robot->getTranslationalTarget() + distance_update);
    double current_distance = this->distanceComputer();
    if (current_distance > 0) {
        robot->setRotationalTarget(robot->getRotationalPosition().fromUnwrapped((pos-robot->getCurrentPosition()).getVectorAngle()));
    }else {
        robot->setRotationalTarget(robot->getRotationalPosition().fromUnwrapped((pos-robot->getCurrentPosition()).getVectorAngle() + Angle::fromDegrees(180)));
    }
    double distance = (pos-robot->getCurrentPosition()).getDistance();
    if (distance < 10) {
        robot->setDoneAngular(true);
    }else {
        robot->setDoneAngular(false);
    }
    if (distance < 5) {
        if (ramp_data.endSpeed == 0) {
            robot->setTranslationalTarget(robot->getTranslationalPosition());
        }
        done = true;
    }
}

template<typename T>
void PositionTarget<T>::on_done() {
    if (ramp_data.endSpeed == 0)
        robot->setTranslationalTarget(robot->getTranslationalPosition());
    robot->setRotationalTarget(robot->getRotationalPosition());
    robot->setDoneDistance(true);
    robot->setDoneAngular(true);
}
template<typename T>
void PositionTarget<T>::reInitAfterStop() {
    ramp->stop();
}





