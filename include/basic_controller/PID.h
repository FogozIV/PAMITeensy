//
// Created by fogoz on 24/04/2025.
//

#ifndef PID_H
#define PID_H

#include <functional>
#include <memory>
#include "BasicController.h"

class BaseRobot;

class PID : public BasicController{

    std::shared_ptr<BaseRobot> robot;
    double kp;
    double ki;
    double kd;

    double old_error = 0;
    double iTerm = 0;

    std::function<double(double)> anti_windup;
    public:
        PID(std::shared_ptr<BaseRobot> robot, double kp, double ki, double kd, std::function<double(double)> f = [](double d){return d;});

        double evaluate(double error) override;

        void setKP(double kp);

        void setKI(double ki);

        void setKD(double kd);

        void setAntiWindup(std::function<double(double)> anti_windup);

        void reset(double error) override;

};

#endif //PID_H
