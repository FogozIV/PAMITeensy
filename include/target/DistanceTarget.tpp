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
    distance = robot->getTranslationalTarget() + distance;
    this->distanceComputer = [this] {
        return distance - robot->getTranslationalPosition();
    };
    ramp = std::make_shared<T>(robot, ramp_data, this->distanceComputer);
    assert(ramp != nullptr);
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
    robot->setPositionTarget(robot->getPositionTarget() + distance_update * robot->getCurrentPosition().getSinCosAngle());
    double distance = this->distanceComputer();
    //less than a mm
    previous_trans_pos = robot->getTranslationalPosition();
    if (abs(distance) < 5 || (abs(robot->getTranslationalPosition() - robot->getTranslationalTarget()) < 1*robot->getDT() && distance_update == 0.0)) {
        done_tick++;
        if(done_tick >= 30){
            robot->setTranslationalTarget(robot->getTranslationalTarget()-distance_update);
            done = true;
        }
    }
    BaseTarget::process();
}

template<typename T>
void DistanceTarget<T>::on_done() {
    robot->setRotationalTarget(robot->getRotationalPosition());
    robot->setDoneDistance(true);
    if (ramp_data.endSpeed == 0.0) {
        robot->setTranslationalTarget(robot->getTranslationalPosition());
        robot->getController()->reset();
    }
}
template<typename T>
void DistanceTarget<T>::reInitAfterStop() {
    ramp->stop();
}

