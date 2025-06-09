//
// Created by fogoz on 08/06/2025.
//

#ifndef TIMEBOUNDQUADRAMP_H
#define TIMEBOUNDQUADRAMP_H
#include "Ramp.h"
#include "functional"
#include "memory"
#include <robot/BaseRobot.h>

#include "utils/BufferFilePrint.h"

class TimeBoundQuadramp : public Ramp{
protected:

    double current_speed = 0;
    double previous_value = 0;
    double t = 0;

    struct {
        double acc_time;
        double dec_time;
        double ste_time;
        double acc_distance;
        double dec_distance;
        double total_distance;
        double ste_speed;
        double initial_speed;
        double end_speed;
    } calculatedData;

    RampData data;
    std::shared_ptr<BaseRobot> robot;
    std::function<double()> fct;

public:
    TimeBoundQuadramp(std::shared_ptr<BaseRobot> robot, RampData data,
                                       std::function<double()> fct);

    void start(double initialSpeed) override;

    double computeAtTime(double t);

    double computeDelta() override;

    double getCurrentSpeed() override;

    void stop() override;
};



#endif //TIMEBOUNDQUADRAMP_H
