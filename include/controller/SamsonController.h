//
// Created by fogoz on 03/08/2025.
//

#ifndef SAMSONCONTROLLER_H
#define SAMSONCONTROLLER_H
#include "BaseController.h"
#include "BasicSpeedController.h"
#include "ControllerFactory.h"
#include "basic_controller/BasicControllerFactory.h"
#include "basic_controller/PID.h"


class BasicController;

class SamsonController : public BasicSpeedController{
protected:
    double k1 = 1;
    double k2 = 1;
    double k3 = 1;
public:
    SamsonController(std::shared_ptr<BaseRobot> robot) : BasicSpeedController(robot) {
        this->type = ControllerFactory::SAMSON;
    }

    SamsonController(std::shared_ptr<BaseRobot> robot, std::shared_ptr<BaseController> controller) : BasicSpeedController(robot) {
        this->type = ControllerFactory::SAMSON;
        switch (controller->getType()) {
            case ControllerFactory::BASE:
                break;
            case ControllerFactory::TRIPLE_BASIC:
                break;
            case ControllerFactory::BASIC_SPEED: {
                auto a = ControllerFactory::castToController<BasicSpeedController>(controller, ControllerFactory::BASIC_SPEED);
                leftWheel = a->getLeftWheelController();
                rightWheel = a->getRightWheelController();
            }
            case ControllerFactory::SAMSON: {
                auto a = ControllerFactory::castToController<SamsonController>(controller, ControllerFactory::SAMSON);
                k1 = a->k1;
                k2 = a->k2;
                k3 = a->k3;
                leftWheel = a->leftWheel;
                rightWheel = a->rightWheel;
            }
        }
    }

    SamsonController(std::shared_ptr<BaseRobot> robot, std::shared_ptr<BasicController> leftWheel, std::shared_ptr<BasicController> rightWheel) : BasicSpeedController(robot, leftWheel, rightWheel) {
        this->type = ControllerFactory::SAMSON;
    }

    SamsonController(std::shared_ptr<BaseRobot> robot, std::shared_ptr<BasicController> leftWheel, std::shared_ptr<BasicController> rightWheel, double k1, double k2, double k3) : BasicSpeedController(robot, leftWheel, rightWheel) {
        this->type = ControllerFactory::SAMSON;
        this->k1 = k1;
        this->k2 = k2;
        this->k3 = k3;
    }

    void registerCommands(CommandParser &parser, const char *name) override;

    void unregisterCommands(CommandParser &parser, const char *name) override;

    void compute() override;

    void reset(bool correct_error) override;

    void serialize(JsonObject json) override;

    std::shared_ptr<BaseController> deserialize(std::shared_ptr<BaseRobot> robot, JsonVariant &json) override;

    template<typename T>
    static std::shared_ptr<T> deserialize_as_T(std::shared_ptr<BaseRobot> robot, const JsonVariant &json);

protected:
    void init(ESCType::ESC type) override;
};


template<typename T>
std::shared_ptr<T> SamsonController::deserialize_as_T(std::shared_ptr<BaseRobot> robot, const JsonVariant &json) {
    std::shared_ptr<T> controller = BasicSpeedController::deserialize_as_T<SamsonController>(robot, json);
    GET_AND_CHECK_JSON(controller, k1, double);
    GET_AND_CHECK_JSON(controller, k2, double);
    GET_AND_CHECK_JSON(controller, k3, double);
    return controller;
}



#endif //SAMSONCONTROLLER_H
