//
// Created by fogoz on 04/06/2025.
//
#pragma once

#include "DistanceTarget.h"
template<typename T>
DistanceTarget<T>::DistanceTarget(std::shared_ptr<BaseRobot> baseRobot, double distance, RampData rampData): BaseTarget(baseRobot),distance(distance), ramp_data(rampData) {
}

template<typename T>
bool DistanceTarget<T>::is_done() {
    return done;
}

template<typename T>
void DistanceTarget<T>::init() {
    streamSplitter.println("DistanceTarget::init");
    robot->setTranslationalTarget(robot->getTranslationalPosition());
    distance = robot->getTranslationalTarget() + distance;
    this->distanceComputer = [this]() {
        return distance - robot->getTranslationalPosition();
    };
    ramp = std::make_shared<T>(robot, ramp_data, this->distanceComputer);
    if (!ramp)
        streamSplitter.println("ramp is null");
    streamSplitter.println(this->distanceComputer());
    ramp->start(robot->getTranslationalRampSpeed());
    robot->setDoneAngular(true);
    robot->setDoneDistance(false);
    streamSplitter.println("DistanceTarget::init done");
}

template<typename T>
void DistanceTarget<T>::process() {
    double distance_update = ramp->computeDelta();
    robot->setTranslationalRampSpeed(ramp->getCurrentSpeed());
    robot->setTranslationalTarget(robot->getTranslationalTarget() + distance_update);
    double distance = this->distanceComputer();
    //less than a mm
    if(distance_update == 0.0 && abs(robot->getTranslationalPosition() - previous_trans_pos) < 1*robot->getDT()){
        done = true;
    }
    previous_trans_pos = robot->getTranslationalPosition();
    if (distance < 5 || (abs(robot->getTranslationalPosition() - robot->getTranslationalTarget()) < 1*robot->getDT() && distance_update == 0.0)) {
        if (ramp_data.endSpeed == 0) {
            robot->setTranslationalTarget(robot->getTranslationalPosition());
        }
        done = true;
    }
    BaseTarget::process();
}

template<typename T>
void DistanceTarget<T>::on_done() {
    if (ramp_data.endSpeed == 0)
        robot->setTranslationalTarget(robot->getTranslationalPosition());
    robot->setRotationalTarget(robot->getRotationalPosition());
    robot->setDoneDistance(true);
}
template<typename T>
void DistanceTarget<T>::reInitAfterStop() {
    ramp->stop();
}

