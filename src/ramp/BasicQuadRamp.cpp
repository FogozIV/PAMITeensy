//
// Created by fogoz on 03/05/2025.
//

#include "ramp/BasicQuadRamp.h"

#include "robot/BaseRobot.h"
void BasicQuadRamp::start(double initialSpeed) {
    currentSpeed = initialSpeed;
}

double BasicQuadRamp::computeDelta() {
    double distance = distanceToPoint();
    double breakingDistance = (pow(currentSpeed, 2) - pow(endSpeed, 2))  / (2 * acc);

    bool isForward = (distance > 0);
    if (isForward) {
        if (distance < breakingDistance * 0.9) {
            currentSpeed = max(endSpeed, currentSpeed - acc * robot->getDT());
        }else {
            currentSpeed = min(maxSpeed, currentSpeed + acc * robot->getDT());
        }
    }else {
        if (distance > breakingDistance * 0.9) {
            currentSpeed = min(endSpeed, currentSpeed + acc * robot->getDT());
        }else {
            currentSpeed = max(maxSpeed, currentSpeed - acc * robot->getDT());
        }
    }
    return currentSpeed * robot->getDT();
}

double BasicQuadRamp::getCurrentSpeed() {
    return currentSpeed;
}

void BasicQuadRamp::stop() {
    currentSpeed = 0.0f;
}

BasicQuadRamp::BasicQuadRamp(std::shared_ptr<BaseRobot> robot, double acc, double maxSpeed,
                             const std::function<double()> &distanceToPoint, double endSpeed) : robot(robot), acc(acc), maxSpeed(maxSpeed), endSpeed(endSpeed), distanceToPoint(distanceToPoint) {

}

BasicQuadRamp::BasicQuadRamp(std::shared_ptr<BaseRobot> robot, RampData data,
    const std::function<double()> &distanceToPoint) : BasicQuadRamp(robot, data.acc, data.maxSpeed, distanceToPoint, data.endSpeed){
}
