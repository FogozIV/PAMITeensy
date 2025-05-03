//
// Created by fogoz on 03/05/2025.
//

#include "ramp/BasicQuadRamp.h"

#include "robot/BaseRobot.h"
void BasicQuadRamp::start(double initialSpeed) {

}

double BasicQuadRamp::computeDelta() {
    return 0;
}

BasicQuadRamp::BasicQuadRamp(std::shared_ptr<BaseRobot> robot, double acc, double dec, double maxSpeed,
                             std::function<double()> distanceToPoint, double endSpeed) {

}
