//
// Created by fogoz on 27/04/2025.
//

#ifndef DYNAMICQUADRAMP_H
#define DYNAMICQUADRAMP_H

#include "Ramp.h"
#include <functional>
#include "../robot/BaseRobot.h"
#include <memory>

class DynamicQuadRamp : public Ramp{
    std::shared_ptr<BaseRobot> robot;
    double acc;
    double dec;
    double maxSpeed;

    double endSpeed;
    double currentSpeed;
    std::function<double()> distanceToPoint;
public:
    DynamicQuadRamp(std::shared_ptr<BaseRobot> robot, double acc, double dec, double maxSpeed, std::function<double()> distanceToPoint, double endSpeed=0);

    void start(double initialSpeed) override;

    double computeDelta() override;
};



#endif //DYNAMICQUADRAMP_H
