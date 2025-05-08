//
// Created by fogoz on 24/04/2025.
//

#ifndef PAMITEENSY_SPEEDESTIMATOR_H
#define PAMITEENSY_SPEEDESTIMATOR_H
#include "Arduino.h"
#include <functional>
#include <memory>

#include "ConditionVariable.h"
//Based on PLLEstimator
//https://discourse.odriverobotics.com/t/rotor-encoder-pll-and-velocity/224/2
//https://github.com/EsialRobotik/asserv_chibios/blob/master/src/Pll.cpp


class BaseRobot;
class SpeedEstimator : public std::enable_shared_from_this<SpeedEstimator> {
    double kp = 0;
    double ki = 0;
    double speed = 0.0f;
    double distance_estimation = 0.0f;
    double real_distance = 0.0f;
    std::shared_ptr<BaseRobot> baseRobot;
    ConditionalVariable bandwidth;
public:
    SpeedEstimator(std::shared_ptr<BaseRobot> baseRobot, double bandwidth);

    void update(double distance);

    void reset();

    void setBandwidth(double bandwidth);

    double getSpeed() const;

    double getDistanceEstimation() const;

    double getRealDistance() const;

    double getBandwidth() const;

    ConditionalVariable& getBandwidthRef();

};


#endif //PAMITEENSY_SPEEDESTIMATOR_H
