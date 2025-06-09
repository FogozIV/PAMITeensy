//
// Created by fogoz on 27/04/2025.
//

#include "ramp/DynamicQuadRamp.h"


DynamicQuadRamp::DynamicQuadRamp(std::shared_ptr<BaseRobot> robot, RampData ramp, std::function<double()> distanceToPoint) : robot(robot), acc(ramp.acc), dec(ramp.dec), maxSpeed(ramp.maxSpeed),endSpeed(ramp.endSpeed), distanceToPoint(distanceToPoint) {

}

void DynamicQuadRamp::start(double initialSpeed) {
    this->currentSpeed = initialSpeed;
}

double DynamicQuadRamp::computeDelta() {
    double distance = distanceToPoint();
    double dt = robot->getDT();


    if(distance < this->currentSpeed * dt) {

    }
    return 0.0f;
}

double DynamicQuadRamp::getCurrentSpeed() {
    return currentSpeed;
}

void DynamicQuadRamp::stop() {
    this->currentSpeed = 0.0f;
}
