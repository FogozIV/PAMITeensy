//
// Created by fogoz on 03/08/2025.
//

#ifndef BASICSPEEDCONTROLLER_H
#define BASICSPEEDCONTROLLER_H
#include <memory>

#include "BaseController.h"
#include "basic_controller/BasicController.h"
#include "basic_controller/BasicControllerFactory.h"


class BasicSpeedController : public BaseController{
protected:
    std::shared_ptr<BasicController> leftWheel;
    std::shared_ptr<BasicController> rightWheel;
    std::shared_ptr<BaseRobot> robot;

public:
    BasicSpeedController(std::shared_ptr<BaseRobot> robot) {
        this->type = ControllerFactory::BASIC_SPEED;
        this->robot = robot;
        this->leftWheel = std::make_shared<PID>(robot, 20, 0,0,500);
        this->rightWheel = std::make_shared<PID>(robot, 20, 0,0,500);
    }

    BasicSpeedController(std::shared_ptr<BaseRobot> robot, std::shared_ptr<BasicController> leftWheel, std::shared_ptr<BasicController> rightWheel) {
        this->type = ControllerFactory::BASIC_SPEED;
        this->robot = robot;
        this->leftWheel = leftWheel;
        this->rightWheel = rightWheel;
    }

    void serialize(JsonObject json) override;

    std::shared_ptr<BaseController> deserialize(std::shared_ptr<BaseRobot> robot, JsonVariant &json) override;

    void registerCommands(CommandParser &parser, const char *name) override;

    void unregisterCommands(CommandParser &parser, const char *name) override;

    void compute() override;

    void reset(bool correct_error) override;

    [[nodiscard]] std::shared_ptr<BasicController> getLeftWheel() const {
        return leftWheel;
    }

    [[nodiscard]] std::shared_ptr<BasicController> getRightWheel() const {
        return rightWheel;
    }


    template<typename T>
    static std::shared_ptr<T> deserialize_as_T(std::shared_ptr<BaseRobot> robot, const JsonVariant &json);

protected:
    void init(ESCType::ESC type) override;
};

template<typename T>
std::shared_ptr<T> BasicSpeedController::deserialize_as_T(std::shared_ptr<BaseRobot> robot, const JsonVariant &json) {
    std::shared_ptr<T> controller = std::make_shared<T>(robot);
    if (json["left_wheel_controller"].is<JsonObject>()) {
        controller->leftWheel = BasicControllerDeserialisation::getFromJson(robot, json["left_wheel_controller"].as<JsonObject>());
    }
    if (json["right_wheel_controller"].is<JsonObject>()) {
        controller->rightWheel = BasicControllerDeserialisation::getFromJson(robot, json["right_wheel_controller"].as<JsonObject>());
    }
    return controller;
}


#endif //BASICSPEEDCONTROLLER_H
