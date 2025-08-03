//
// Created by fogoz on 03/08/2025.
//

#include <controller/BasicSpeedController.h>

void BasicSpeedController::serialize(JsonObject json) {
    leftWheel->serialize(json["left_wheel_controller"].to<JsonObject>());
    rightWheel->serialize(json["right_wheel_controller"].to<JsonObject>());
}

std::shared_ptr<BaseController> BasicSpeedController::deserialize(std::shared_ptr<BaseRobot> robot, JsonVariant &json) {
    return BasicSpeedController::deserialize_as_T<BasicSpeedController>(robot, json);
}

void BasicSpeedController::registerCommands(CommandParser &parser, const char *name) {
    leftWheel->registerCommands(parser, "left_wheel");
    rightWheel->registerCommands(parser, "right_wheel");
}

void BasicSpeedController::unregisterCommands(CommandParser &parser, const char *name) {
    leftWheel->unregisterCommands(parser, "left_wheel");
    rightWheel->unregisterCommands(parser, "right_wheel");
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
    initVariable(leftWheel->variables, leftWheel->frequency, leftWheel->alpha, leftWheel->gamma, leftWheel->low_bound, leftWheel->high_bound);
    auto frequency = rightWheel->frequency;
    for (auto& f : frequency) {
        f *= 1.27;
    }
    initVariable(rightWheel->variables, rightWheel->frequency, rightWheel->alpha, rightWheel->gamma, rightWheel->low_bound, rightWheel->high_bound);
}
