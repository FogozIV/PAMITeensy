//
// Created by fogoz on 24/04/2025.
//

#ifndef PID_H
#define PID_H

#include <functional>
#include <memory>
#include "BasicController.h"
#include "ArduinoJson.h"
class BaseRobot;
class PID : public BasicController{

    std::shared_ptr<BaseRobot> robot;
    double kp;
    double ki;
    double kd;

    double old_error = 0;
    double iTerm = 0;
    double anti_windup;
public:
    PID(std::shared_ptr<BaseRobot> robot, double kp, double ki, double kd, double anti_windup);

    double evaluate(double error) override;

    void setKP(double kp);

    void setKI(double ki);

    void setKD(double kd);

    void setAntiWindup(double anti_windup);

    void reset(double error) override;

    double getKd() const;

    double getKi() const;

    double getKp() const;

    double getAntiWindup() const;

};


inline std::shared_ptr<PID> getPIDFromJson(std::shared_ptr<BaseRobot> robot, JsonVariantConst src){
    double kp;
    double ki;
    double kd;
    double anti_windup;
    if(src["kp"].is<double>()){
        kp = src["kp"].as<double>();
    }
    if(src["ki"].is<double>()){
        ki = src["ki"].as<double>();
    }
    if(src["kd"].is<double>()){
        kd = src["kd"].as<double>();
    }
    if(src["anti_windup"].is<double>()){
        anti_windup = src["anti_windup"].as<double>();
    }
    return std::make_shared<PID>(robot, kp, ki, kd, anti_windup);
}

inline void PIDtoJson(std::shared_ptr<PID> pid, JsonVariant dst){
    JsonObject variant = dst.to<JsonObject>();
    variant["kp"] = pid->getKp();
    variant["ki"] = pid->getKi();
    variant["kd"] = pid->getKd();
    variant["anti_windup"] = pid->getAntiWindup();
}

#endif //PID_H
