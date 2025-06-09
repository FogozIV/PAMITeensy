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
    Serial.printf("Distance to target %f\r\n", distance);
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
    Serial.printf("Delta send %f \r\n", currentSpeed*robot->getDT());
    return currentSpeed * robot->getDT();
}

double BasicQuadRamp::getCurrentSpeed() {
    return currentSpeed;
}

void BasicQuadRamp::stop() {
    currentSpeed = 0.0f;
}

BasicQuadRamp::BasicQuadRamp(std::shared_ptr<BaseRobot> robot, RampData ramp,
                             const std::function<double()> &distanceToPoint) : robot(robot), acc(ramp.acc), maxSpeed(ramp.maxSpeed), endSpeed(ramp.endSpeed), distanceToPoint(distanceToPoint) {

}
