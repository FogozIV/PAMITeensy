//
// Created by fogoz on 24/04/2025.
//

#include <utility>

#include "basic_controller/PID.h"
#include "robot/BaseRobot.h"

PID::PID(std::shared_ptr<BaseRobot> robot, double kp, double ki, double kd, double anti_windup) : robot(std::move(robot)), kp(kp), ki(ki), kd(kd), anti_windup(anti_windup) {
    type = BasicControllerType::PID;
}

PID::PID(std::shared_ptr<BaseRobot> robot): robot(std::move(robot)), kp(0), ki(0), kd(0), anti_windup(0) {
    type = BasicControllerType::PID;
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


void PID::serialize( JsonObject json) {
    SET_JSON(ki);
    SET_JSON(kp);
    SET_JSON(anti_windup);
    SET_JSON(type);
    SET_JSON(kd);
}


std::shared_ptr<BasicController> PID::deserialize(std::shared_ptr<BaseRobot> robot, JsonVariant &json) {
    return deserialize_as_T<PID>(robot, json);
}
#define COMMAND_PID(name, variable) \
SUB_COMMAND_PID(name, KP, (variable)->getKpRef())\
SUB_COMMAND_PID(name, KI, (variable)->getKiRef())\
SUB_COMMAND_PID(name, KD, (variable)->getKdRef())\
SUB_COMMAND_PID(name, anti_windup, (variable)->getAntiWindupRef())

#define SUB_COMMAND_PID(name, sub_name, variable){ \
    std::string command_name = std::string("pid_") + std::string(name) + std::string("_"#sub_name); \
    std::string comment = std::string("change value or look at the value  of PID ") + std::string(name) +std::string(" "#sub_name);\
    parser.registerMathCommand(command_name, variable, [name](Stream& stream, double value, MathOP op){ \
    std::string function_return = std::string("La valeur du PID ") + std::string(name) +std::string(" "#sub_name" est : %f\r\n");\
    stream.printf(function_return.c_str(), value);\
    return "";\
    }, comment.c_str());}
void PID::registerCommands(CommandParser &parser, const char* name) {
    COMMAND_PID(name, this)
}
#undef SUB_COMMAND_PID
#define SUB_COMMAND_PID(name, sub_name, variable) \
    parser.removeAllCommands(std::string("pid_") + (name) + "_"#sub_name);
void PID::unregisterCommands(CommandParser &parser, const char* name) {
    COMMAND_PID(name, this)
}
#undef SUB_COMMAND_PID
#undef COMMAND_PID