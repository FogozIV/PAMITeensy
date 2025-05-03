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
    double dec;
    double maxSpeed;

    double endSpeed;
    double currentSpeed;
    std::function<double()> distanceToPoint;
public:
    BasicQuadRamp(std::shared_ptr<BaseRobot> robot, double acc, double dec, double maxSpeed, std::function<double()> distanceToPoint, double endSpeed=0);

    void start(double initialSpeed) override;

    double computeDelta() override;

};


#endif //PAMITEENSY_BASICQUADRAMP_H
