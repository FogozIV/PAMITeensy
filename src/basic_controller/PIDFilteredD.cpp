//
// Created by fogoz on 13/06/2025.
//

#include <basic_controller/PIDFilteredD.h>

#include "robot/BaseRobot.h"

PIDFilteredD::PIDFilteredD(std::shared_ptr<BaseRobot> robot, double kp, double ki, double kd, double anti_windup,
                           double N): PID(robot, kp, ki, kd, anti_windup), N(N) {
    this->type = BasicControllerType::PIDFilteredD;
}

PIDFilteredD::PIDFilteredD(std::shared_ptr<BaseRobot> robot, const std::shared_ptr<PID>& pid): PID(robot, pid) {
    this->type = BasicControllerType::PIDFilteredD;
}

//From the course note DiscreteTimePIDController.pdf we replaced T by N and Ts by the current dt
double PIDFilteredD::evaluate(double error) {
    double result = 0;
    double dt = robot->getDT();
    double alpha = 1.0 / (1.0 + N / dt);
    double de = error - old_error;
    result += kp * error;
    uP = kp * error;
    iTerm += ki * error * dt;
    iTerm = max(min(iTerm, anti_windup), -anti_windup);
    uI = iTerm;
    rawUd = kd * de/dt;
    uD = N/dt * alpha * uD + rawUd * alpha;
    result += uD;
    result += iTerm;
    this->old_error = error;
    return result;
}

double PIDFilteredD::simulate(double error) const {
    double result = 0;
    double iTerm = this->iTerm;
    double dt = robot->getDT();
    double alpha = N / (1.0 + N*dt);
    double de = error - old_error;
    result += kp * error;
    iTerm += ki * error * dt;
    iTerm = max(min(iTerm, anti_windup), -anti_windup);
    double uD = alpha * this->uD + kd * alpha * de;
    result += uD;
    result += iTerm;
    return result;
}

double & PIDFilteredD::getNRef() {
    return N;
}

void PIDFilteredD::serialize(JsonObject json) {
    PID::serialize(json);
    SET_JSON(N);
}

double PIDFilteredD::getRawUd() const {
    return rawUd;
}

std::shared_ptr<BasicController> PIDFilteredD::deserialize(std::shared_ptr<BaseRobot> robot, JsonVariant &json) {
    return deserialize_as_T<PIDFilteredD>(robot, json);
}
#define COMMAND_PID(name, variable) \
SUB_COMMAND_PID(name, N, (variable)->getNRef())
#define SUB_COMMAND_PID(name, sub_name, variable){ \
std::string command_name = std::string("pid_") + std::string(name) + std::string("_"#sub_name); \
std::string comment = std::string("change value or look at the value  of PID ") + std::string(name) +std::string(" "#sub_name);\
parser.registerMathCommand(command_name, variable, [name](Stream& stream, double value, MathOP op){ \
std::string function_return = std::string("La valeur du PID ") + std::string(name) +std::string(" "#sub_name" est : %f\r\n");\
stream.printf(function_return.c_str(), value);\
return "";\
}, comment);}
void PIDFilteredD::registerCommands(CommandParser &parser, const char *name) {
    PID::registerCommands(parser, name);
    COMMAND_PID(name, this)
}
#undef SUB_COMMAND_PID

#define SUB_COMMAND_PID(name, sub_name, variable) \
parser.removeAllCommands(std::string("pid_") + std::string(name) + std::string("_"#sub_name));

void PIDFilteredD::unregisterCommands(CommandParser &parser, const char *name) {
    PID::unregisterCommands(parser, name);
    COMMAND_PID(name, this)
}

void PIDFilteredD::reset(double error) {
    PID::reset(error);
    uD= 0;
}
