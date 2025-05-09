//
// Created by fogoz on 08/05/2025.
//

#ifndef CALCULATEDQUADRAMP_H
#define CALCULATEDQUADRAMP_H
#include <memory>

#include "Ramp.h"
#include "robot/BaseRobot.h"

struct CalculatedQuadrampData {
    double initial_speed;
    double end_speed;
    double acc_time;
    double dec_time;
    double acc_distance;
    double dec_distance;
    double ste_speed;
    double ste_time;

    bool inversed;
    double initial_distance;
};

class CalculatedQuadramp : public Ramp{
    std::shared_ptr<BaseRobot> robot;
    RampData data;
    std::function<double()> distanceToPoint;
    CalculatedQuadrampData calculatedData;
    double previous_value = 0;
    double t = 0;
    double current_speed = 0;
public:
    CalculatedQuadramp(std::shared_ptr<BaseRobot> robot, RampData data, std::function<double()> distanceToPoint);

    double computeAtTime(double t);

    void start(double initialSpeed) override;

    double computeDelta() override;

    double getCurrentSpeed() override;

    void stop() override;

    ~CalculatedQuadramp() override;
};



#endif //CALCULATEDQUADRAMP_H
