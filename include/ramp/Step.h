//
// Created by fogoz on 23/07/2025.
//

#ifndef STEP_H
#define STEP_H
#include <functional>
#include <memory>

#include "Ramp.h"
#include "robot/BaseRobot.h"


class Step : public Ramp{
    bool sent = false;
    std::function<double()> distance;
    double distance_send = 0;
public:
    Step(std::shared_ptr<BaseRobot> robot, RampData ramp_data, std::function<double()> distance);

    void start(double initialSpeed) override;

    double computeDelta() override;

    double getCurrentSpeed() override;

    void stop() override;
};



#endif //STEP_H
