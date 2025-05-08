//
// Created by fogoz on 03/05/2025.
//

#ifndef PAMITEENSY_BASICQUADRAMP_H
#define PAMITEENSY_BASICQUADRAMP_H

#include "Ramp.h"
#include <memory>
#include <functional>
class BaseRobot;
class BasicQuadRamp : public Ramp {
    std::shared_ptr<BaseRobot> robot;
    double acc;
    double maxSpeed;

    double endSpeed;
    double currentSpeed = 0;
    std::function<double()> distanceToPoint;
public:
    BasicQuadRamp(std::shared_ptr<BaseRobot> robot, double acc, double maxSpeed, const std::function<double()> &distanceToPoint, double endSpeed=0);

    BasicQuadRamp(std::shared_ptr<BaseRobot> robot, RampData data, const std::function<double()> &distanceToPoint);

    void start(double initialSpeed) override;

    double computeDelta() override;

    double getCurrentSpeed() override;

    void stop() override;
};


#endif //PAMITEENSY_BASICQUADRAMP_H
