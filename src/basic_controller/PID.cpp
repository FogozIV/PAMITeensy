//
// Created by fogoz on 24/04/2025.
//

#include <utility>

#include "basic_controller/PID.h"
#include "robot/BaseRobot.h"

PID::PID(std::shared_ptr<BaseRobot> robot, double kp, double ki, double kd, std::function<double(double)> anti_windup) : robot(std::move(robot)), kp(kp), ki(ki), kd(kd), anti_windup(std::move(anti_windup)) {

}


double PID::evaluate(double error){
	double result = 0;
	result += kp * error;
	iTerm += ki * error * robot->getDT();
	iTerm = anti_windup(iTerm);
	result += kd * (error - old_error)/robot->getDT();
	result += iTerm;
	this->old_error = error;
	return result;
}

void PID::setKP(double kp){
    this->kp = kp;
}

void PID::setKI(double ki){
    this->ki = ki;
}
void PID::setKD(double kd){
    this->kd = kd;
}
void PID::setAntiWindup(std::function<double(double)> anti_windup){
    this->anti_windup = std::move(anti_windup);
}
void PID::reset(double error){
    this->old_error = error;
    this->iTerm = 0.0;
}
