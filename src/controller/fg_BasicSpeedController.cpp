//
// Created by fogoz on 03/08/2025.
//

#include <controller/BasicSpeedController.h>

#include "basic_controller/PIDFilteredD.h"
#include "basic_controller/PIDSpeedFeedForward.h"

void BasicSpeedController::serialize(JsonObject json) {
    leftWheel->serialize(json["left_wheel_controller"].to<JsonObject>());
    rightWheel->serialize(json["right_wheel_controller"].to<JsonObject>());
}

std::shared_ptr<BaseController> BasicSpeedController::deserialize(std::shared_ptr<BaseRobot> robot, JsonVariant &json) {
    return BasicSpeedController::deserialize_as_T<BasicSpeedController>(robot, json);
}
#define CHANGE_CONTROLLER(name, fft) \
switch(args[0].asUInt64() + 1){ /* To ignore BasicController*/\
case BasicControllerType::PID:\
stream.println("Changing type of controller to PID"); \
this->set##name##Controller(std::make_shared<PID>(robot, BasicControllerDeserialisation::castToPID(get##name##Controller()))); \
break;\
case BasicControllerType::PIDSpeedFeedForward:\
stream.println("Changing type of controller to PID Feed forward");\
set##name##Controller(std::make_shared<PIDSpeedFeedForward>(robot, BasicControllerDeserialisation::castToPID(get##name##Controller()))); \
break;\
case BasicControllerType::PIDFilteredD:\
stream.println("Changing type of controller to PID filtered");\
set##name##Controller(std::make_shared<PIDFilteredD>(robot, BasicControllerDeserialisation::castToPID(get##name##Controller()))); \
break;\
case BasicControllerType::FeedForward:\
stream.println("Changing type of controller to Feed Forward wrapper");\
set##name##Controller(std::make_shared<FeedForward>(robot, get##name##Controller(), 1, FeedForwardType::fft)); \
break;\
default:\
stream.printf("Unknown type %u\r\n", args[0].asUInt64());\
break;\
}
#define TEXT_CONTROLLER(name)\
"Allows to change the controller "#name "to 0 = PID, 1= PID Feed Forward 2= PID Filtered D 3= Feed Forward wrapper"
void BasicSpeedController::registerCommands(CommandParser &parser, const char *name) {
    leftWheel->registerCommands(parser, "left_wheel");
    rightWheel->registerCommands(parser, "right_wheel");

    parser.registerCommand("change_left_wheel_to", "u", [this](std::vector<CommandParser::Argument> args, Stream& stream){
        CHANGE_CONTROLLER(LeftWheel, DISTANCE)
        return "";
    }, TEXT_CONTROLLER(left_wheel));

    parser.registerCommand("change_right_wheel_to", "u", [this](std::vector<CommandParser::Argument> args, Stream& stream){
        CHANGE_CONTROLLER(RightWheel, DISTANCE)
        return "";
    }, TEXT_CONTROLLER(right_wheel));

}

void BasicSpeedController::unregisterCommands(CommandParser &parser, const char *name) {
    leftWheel->unregisterCommands(parser, "left_wheel");
    rightWheel->unregisterCommands(parser, "right_wheel");

    parser.removeAllCommands("change_left_wheel_to");
    parser.removeAllCommands("change_right_wheel_to");

}

void FASTRUN BasicSpeedController::compute() {
    robot->lockMotorMutex();
    auto leftSpeedTarget = robot->getTranslationalRampSpeed() - robot->getRotationalRampSpeed().toRadians() * robot->getWheelPositionManagerParameters()->track_mm/2;
    auto rightSpeedTarget = robot->getTranslationalRampSpeed() + robot->getRotationalRampSpeed().toRadians() * robot->getWheelPositionManagerParameters()->track_mm/2;

    auto left = leftWheel->evaluate(leftSpeedTarget - robot->getState(KalmanFilter::v_L_wheel));
    auto right = rightWheel->evaluate(rightSpeedTarget - robot->getState(KalmanFilter::v_R_wheel));

    robot->getLeftMotor()->setPWM(left);
    robot->getRightMotor()->setPWM(right);


    robot->unlockMotorMutex();
}

void BasicSpeedController::reset(bool correct_error) {
    leftWheel->reset();
    rightWheel->reset();
}

void BasicSpeedController::init(ESCType::ESC type) {
    std::vector<double> frequencies = {0.45, 0.465, 0.48, 0.495, 0.51, 0.525, 0.54, 0.555};
    std::vector<double> frq(frequencies.begin(), frequencies.begin() + leftWheel->variables.size());
    initVariable(leftWheel->variables, frq, leftWheel->alpha, leftWheel->gamma, leftWheel->low_bound, leftWheel->high_bound);
    frq = std::vector<double>(frequencies.begin() + leftWheel->variables.size(), frequencies.begin() + leftWheel->variables.size() + rightWheel->variables.size());
    initVariable(rightWheel->variables, frq, rightWheel->alpha, rightWheel->gamma, rightWheel->low_bound, rightWheel->high_bound);
}
