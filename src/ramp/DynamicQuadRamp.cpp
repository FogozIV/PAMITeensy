//
// Created by fogoz on 27/04/2025.
//

#include "ramp/DynamicQuadRamp.h"


DynamicQuadRamp::DynamicQuadRamp(std::shared_ptr<BaseRobot> robot, double acc, double dec, double maxSpeed, std::function<double()> distanceToPoint, double endSpeed) : robot(robot), acc(acc), dec(dec), maxSpeed(maxSpeed),endSpeed(endSpeed), distanceToPoint(distanceToPoint) {

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
