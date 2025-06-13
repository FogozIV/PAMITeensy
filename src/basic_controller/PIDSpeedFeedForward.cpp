//
// Created by fogoz on 08/06/2025.
//

#include <basic_controller/PIDSpeedFeedForward.h>

#include "packets/PacketDefinition.h"
#include "robot/BaseRobot.h"

PIDSpeedFeedForward::PIDSpeedFeedForward(const std::shared_ptr<BaseRobot> &robot, double kp, double ki, double kd,
                                         double anti_windup, double ff_gain, PIDSpeedFeedForwardType::FeedForward type): PID(robot, kp, ki, kd, anti_windup), ff_gain(ff_gain), feedforward_type(type) {
    this->type = BasicControllerType::PIDSpeedFeedForward;
    speedFromFeedForward();
}

PIDSpeedFeedForward::PIDSpeedFeedForward(const std::shared_ptr<BaseRobot> &robot, const std::shared_ptr<PID> &pid) : PID(robot, pid), ff_gain(0.0), get_speed(nullptr) {
    this->type = BasicControllerType::PIDSpeedFeedForward;
}

double PIDSpeedFeedForward::evaluate(double error) {
    uFF = ff_gain * get_speed();
    if (get_speed != nullptr) {
        return PID::evaluate(error) + ff_gain * get_speed();
    }
    return PID::evaluate(error);
}

double PIDSpeedFeedForward::simulate(double error) const {
    if (get_speed != nullptr) {
        return PID::simulate(error)+ ff_gain * get_speed();
    }
    return PID::simulate(error);
}

double & PIDSpeedFeedForward::getFeedForwardRef() {
    return ff_gain;
}

std::shared_ptr<PIDSpeedFeedForward> PIDSpeedFeedForward::fromPID(std::shared_ptr<PID> pid, double ff_gain,std::shared_ptr<BaseRobot> robot, PIDSpeedFeedForwardType::FeedForward type) {
    if (pid->getType() == BasicControllerType::PIDSpeedFeedForward) {
        auto pidff = std::static_pointer_cast<PIDSpeedFeedForward>(pid);
        return std::make_shared<PIDSpeedFeedForward>(robot, pid->getKp(), pid->getKi(), pid->getKd(), pid->getAntiWindup(), pidff->getFeedForwardRef(), pidff->feedforward_type);
    }
    return std::make_shared<PIDSpeedFeedForward>(robot, pid->getKp(), pid->getKi(), pid->getKd(), pid->getAntiWindup(), ff_gain, type);
}

void PIDSpeedFeedForward::serialize(JsonObject json) {
    PID::serialize(json);
    SET_JSON(ff_gain);
    SET_JSON(feedforward_type);
}

std::shared_ptr<BasicController> PIDSpeedFeedForward::deserialize(std::shared_ptr<BaseRobot> robot, JsonVariant &json) {
    return deserialize_as_T<PIDSpeedFeedForward>(robot, json);
}

void PIDSpeedFeedForward::speedFromFeedForward() {
    auto robot = this->robot;
    switch (feedforward_type) {
        case PIDSpeedFeedForwardType::DISTANCE:
            get_speed = [robot]() {return robot->getTranslationalRampSpeed();};
            break;
        case PIDSpeedFeedForwardType::ANGLE:
            get_speed = [robot]() {return robot->getRotationalRampSpeed().toDegrees();};
            break;
    }
}
#define COMMAND_PID(name, variable) \
SUB_COMMAND_PID(name, FEED_FORWARD, (variable)->getFeedForwardRef())
#define SUB_COMMAND_PID(name, sub_name, variable){ \
std::string command_name = std::string("pid_") + std::string(name) + std::string("_"#sub_name); \
std::string comment = std::string("change value or look at the value  of PID ") + std::string(name) +std::string(" "#sub_name);\
parser.registerMathCommand(command_name, variable, [name](Stream& stream, double value, MathOP op){ \
std::string function_return = std::string("La valeur du PID ") + std::string(name) +std::string(" "#sub_name" est : %f\r\n");\
stream.printf(function_return.c_str(), value);\
return "";\
}, comment);}
void PIDSpeedFeedForward::registerCommands(CommandParser &parser, const char* name) {
    PID::registerCommands(parser, name);
    COMMAND_PID(name, this)
}
#undef SUB_COMMAND_PID
#define SUB_COMMAND_PID(name, sub_name, variable) \
parser.removeAllCommands(std::string("pid_") + std::string(name) + std::string("_"#sub_name));

void PIDSpeedFeedForward::unregisterCommands(CommandParser &parser, const char* name) {
    PID::unregisterCommands(parser, name);
    COMMAND_PID(name, this)
}

double PIDSpeedFeedForward::getUff() const {
    return uFF;
}

#undef SUB_COMMAND_PID
#undef COMMAND_PID


