//
// Generic feed forward wrapper for BasicController
//
#ifndef BASICFEEDFORWARD_H
#define BASICFEEDFORWARD_H

#include "BasicController.h"
#include <functional>
#include <memory>

class BaseRobot;
namespace BasicControllerDeserialisation {
    std::shared_ptr<BasicController> getFromJson(std::shared_ptr<BaseRobot> robot,
                                                 const JsonVariant &json);
}
namespace FeedForwardType {
    enum FeedForward {
        DISTANCE,
        ANGLE
    };
}

class FeedForward : public BasicController {
protected:
    std::shared_ptr<BasicController> controller;
    std::function<double()> get_speed;
    double ff_gain = 0.0;
    FeedForwardType::FeedForward feedforward_type = FeedForwardType::DISTANCE;
    double uFF = 0;
    std::shared_ptr<BaseRobot> robot;
public:
    FeedForward(std::shared_ptr<BaseRobot> robot,
                std::shared_ptr<BasicController> controller,
                double ff_gain = 0.0,
                FeedForwardType::FeedForward type = FeedForwardType::DISTANCE);

    double evaluate(double error) override;
    double simulate(double error) const override;
    void reset(double error) override { controller->reset(error); }

    double getUff() const { return uFF; }

    double& getFeedForwardRef();
    std::shared_ptr<BasicController> getInnerController() const { return controller; }

    void serialize(JsonObject json) override;
    std::shared_ptr<BasicController> deserialize(std::shared_ptr<BaseRobot> robot, JsonVariant &json) override;

    template<typename T>
    static std::shared_ptr<T> deserialize_as_T(std::shared_ptr<BaseRobot> robot, const JsonVariant &json);

    void registerCommands(CommandParser &parser, const char* name) override;
    void unregisterCommands(CommandParser &parser, const char* name) override;
    void multiply(double d) override;

protected:
    void speedFromFeedForward();
};

template<typename T>
std::shared_ptr<T> FeedForward::deserialize_as_T(std::shared_ptr<BaseRobot> robot, const JsonVariant &json) {
    std::shared_ptr<T> p = std::make_shared<T>(robot, nullptr);
    if(json["controller"].is<JsonVariantConst>()) {
        p->controller = BasicControllerDeserialisation::getFromJson(robot, json["controller"]);
    }
    GET_AND_CHECK_JSON(p, ff_gain, double);
    GET_AND_CHECK_JSON(p, feedforward_type, FeedForwardType::FeedForward);
    p->speedFromFeedForward();
    return p;
}

#endif // BASICFEEDFORWARD_H
