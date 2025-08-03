//
// Created by fogoz on 24/04/2025.
//

#include <utility>

#include "basic_controller/PID.h"
#include "robot/BaseRobot.h"

PID::PID(std::shared_ptr<BaseRobot> robot, double kp, double ki, double kd, double anti_windup) : robot(std::move(robot)), kp(kp), ki(ki), kd(kd), anti_windup(anti_windup) {
    type = BasicControllerType::PID;
    variables.emplace_back(&this->kp);
    variables.emplace_back(&this->ki);
    variables.emplace_back(&this->kd);
    std::vector<double> frequencies = {0.45, 0.465, 0.48};
    std::vector<double> alpha = {0.05,0.05,0.05};
    std::vector<double> gamma = {0.01,0.01,0.01};
    std::vector<double> low_bound = {1,0.0001,0.0001};
    std::vector<double> high_bound = {200,1000,200};
    frequency.insert(frequency.end(), frequencies.begin(), frequencies.end());
    BasicController::alpha.insert(BasicController::alpha.end(), alpha.begin(), alpha.end());
    BasicController::gamma.insert(BasicController::gamma.end(), gamma.begin(), gamma.end());
    BasicController::low_bound.insert(BasicController::low_bound.end(), low_bound.begin(), low_bound.end());
    BasicController::high_bound.insert(BasicController::high_bound.end(), high_bound.begin(), high_bound.end());
}

PID::PID(std::shared_ptr<BaseRobot> robot, const std::shared_ptr<PID>& pid): robot(std::move(robot)), kp(0), ki(0), kd(0), anti_windup(0) {
    variables.emplace_back(&this->kp);
    variables.emplace_back(&this->ki);
    variables.emplace_back(&this->kd);
    type = BasicControllerType::PID;
    if (pid != nullptr) {
        kp = pid->kp;
        ki = pid->ki;
        kd = pid->kd;
        anti_windup = pid->anti_windup;
    }
    std::vector<double> frequencies = {0.2, 0.31, 0.47};
    std::vector<double> alpha = {0.5,0.5,0.5};
    std::vector<double> gamma = {0.2,0.2,0.2};
    std::vector<double> low_bound = {1,0.0001,0.0001};
    std::vector<double> high_bound = {200,1000,200};
    frequency.insert(frequency.end(), frequencies.begin(), frequencies.end());
    BasicController::alpha.insert(BasicController::alpha.end(), alpha.begin(), alpha.end());
    BasicController::gamma.insert(BasicController::gamma.end(), gamma.begin(), gamma.end());
    BasicController::low_bound.insert(BasicController::low_bound.end(), low_bound.begin(), low_bound.end());
    BasicController::high_bound.insert(BasicController::high_bound.end(), high_bound.begin(), high_bound.end());
}


double FASTRUN PID::evaluate(double error){
	double result = 0;
	result += kp * error;
    uP = kp * error;
	iTerm += ki * error * robot->getDT();
	iTerm = max(min(iTerm, anti_windup), -anti_windup);
    uI = iTerm;
    uD = kd * (error - old_error)/robot->getDT();
	result += uD;
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
    }, comment);}
void PID::registerCommands(CommandParser &parser, const char* name) {
    COMMAND_PID(name, this)
}
#undef SUB_COMMAND_PID
#define SUB_COMMAND_PID(name, sub_name, variable) \
    parser.removeAllCommands(std::string("pid_") + std::string(name) + std::string("_"#sub_name));
void PID::unregisterCommands(CommandParser &parser, const char* name) {
    COMMAND_PID(name, this)
}

void PID::multiply(double d) {
    this->kp *= d;
    this->ki *= d;
    this->kd *= d;
}

double PID::getUd() const {
    return uD;
}

double PID::getUi() const {
    return uI;
}

double PID::getUp() const {
    return uP;
}

#undef SUB_COMMAND_PID
#undef COMMAND_PID