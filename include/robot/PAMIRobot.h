//
// Created by fogoz on 24/04/2025.
//

#ifndef PAMITEENSY_PAMIROBOT_H
#define PAMITEENSY_PAMIROBOT_H
#include "robot/BaseRobot.h"
#include "utils/PositionManager.h"
#include "chrono"

class PAMIRobot : public BaseRobot{
    std::vector<std::shared_ptr<BaseTarget>> targets;
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::duration<double, std::ratio<1,1>>> previous_time;
    double dt = 0.005;
public:
    double getDT() override;

    void computeTarget() override;

    void computePosition() override;

    void computeController() override;

    void addTarget(std::shared_ptr<BaseTarget> target) override;

    void compute() override;

    void init(std::shared_ptr<PAMIRobot> robot);
};


#endif //PAMITEENSY_PAMIROBOT_H
