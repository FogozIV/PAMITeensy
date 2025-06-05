//
// Created by fogoz on 24/04/2025.
//

#include <utility>

#include "basic_controller/PID.h"
#include "robot/BaseRobot.h"

PID::PID(std::shared_ptr<BaseRobot> robot, double kp, double ki, double kd, double anti_windup) : robot(std::move(robot)), kp(kp), ki(ki), kd(kd), anti_windup(anti_windup) {

}


double PID::evaluate(double error){
	double result = 0;
	result += kp * error;
	iTerm += ki * error * robot->getDT();
	iTerm = max(min(iTerm, anti_windup), -anti_windup);
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
void PID::setAntiWindup(double anti_windup){
    this->anti_windup = anti_windup;
}
void PID::reset(double error){
    this->old_error = error;
    this->iTerm = 0.0;
}

double PID::getKd() const {
    return kd;
}

double PID::getKi() const {
    return ki;
}

double PID::getKp() const {
    return kp;
}

double & PID::getKdRef() {
    return kd;
}

double & PID::getKiRef() {
    return ki;
}

double & PID::getKpRef() {
    return kp;
}

double & PID::getAntiWindupRef() {
    return anti_windup;
}

double PID::getAntiWindup() const {
    return anti_windup;
}

double PID::simulate(double error) const {
    double result = 0;
    double iTerm = this->iTerm;
    result += kp * error;
    iTerm += ki * error * robot->getDT();
    iTerm = max(min(iTerm, anti_windup), -anti_windup);
    result += kd * (error - old_error)/robot->getDT();
    result += iTerm;
    return result;
}
