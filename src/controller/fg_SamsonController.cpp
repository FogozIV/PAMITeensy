//
// Created by fogoz on 03/08/2025.
//

#include <controller/SamsonController.h>

#include "controller/SimpleTripleBasicController.h"
#define SAMSON_VALUES\
    S_VALUES(k1)\
    S_VALUES(k2)\
    S_VALUES(k3)


#define S_VALUES(name) \
    parser.registerMathCommand("samson_"#name, name, [](Stream& stream, double v, MathOP op){ \
        stream.printf("The new value for the " #name "value is %f\r\n", v);\
        return "";\
    }, "Setting samson controller "#name" value");


void SamsonController::registerCommands(CommandParser &parser, const char *name) {
    BasicSpeedController::registerCommands(parser, name);
    SAMSON_VALUES
}
#undef S_VALUES

#define S_VALUES(name) \
    parser.removeAllCommands("samson_"#name);

void SamsonController::unregisterCommands(CommandParser &parser, const char *name) {
    BasicSpeedController::unregisterCommands(parser, name);
    SAMSON_VALUES
}

void FASTRUN SamsonController::compute() {
    robot->lockMotorMutex();
    auto currentPosition = robot->getCurrentPosition();
    auto targetPosition = robot->getPositionTarget();
    Position pos = targetPosition - currentPosition;
    double e_x = cos(currentPosition.getAngle().toRadians()) * pos.getX() + sin(currentPosition.getAngle().toRadians()) * pos.getY();
    double e_y = -sin(currentPosition.getAngle().toRadians()) * pos.getX() + cos(currentPosition.getAngle().toRadians()) * pos.getY();
    double e_theta = pos.getAngle().toRadians();

    double v = k1 * e_x;
    double w = k2 * e_theta + k3 * e_y;

    double v_left = v - w * robot->getWheelPositionManagerParameters()->track_mm/2;
    double v_right = v + w * robot->getWheelPositionManagerParameters()->track_mm/2;

    //goal is to reach v_left & v_right
    double output_left = leftWheel->evaluate(v_left - robot->getState(KalmanFilter::v_L_wheel));
    double output_right = rightWheel->evaluate(v_right - robot->getState(KalmanFilter::v_R_wheel));
    robot->getLeftMotor()->setPWM(output_left);
    robot->getRightMotor()->setPWM(output_right);
    robot->unlockMotorMutex();
}

void SamsonController::reset(bool correct_error) {
    BasicSpeedController::reset(correct_error);
}

void SamsonController::serialize(JsonObject json) {
    BasicSpeedController::serialize(json);
    SET_JSON(k1);
    SET_JSON(k2);
    SET_JSON(k3);
}

std::shared_ptr<BaseController> SamsonController::deserialize(std::shared_ptr<BaseRobot> robot, JsonVariant &json) {
    return deserialize_as_T<SamsonController>(robot, json);
}

void SamsonController::init(ESCType::ESC type) {
    initVariable(&k1, 0.31);
    initVariable(&k2, 0.53);
    initVariable(&k3, 0.79);
}
