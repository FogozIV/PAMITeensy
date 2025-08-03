//
// Generic feed forward wrapper for BasicController
//
#include <basic_controller/FeedForward.h>
#include <basic_controller/BasicControllerFactory.h>
#include "robot/BaseRobot.h"

FeedForward::FeedForward(std::shared_ptr<BaseRobot> robot,
                         std::shared_ptr<BasicController> controller,
                         double ff_gain,
                         FeedForwardType::FeedForward type)
    : controller(std::move(controller)), ff_gain(ff_gain),
      feedforward_type(type), robot(std::move(robot)) {
    this->type = BasicControllerType::FeedForward;
    variables.emplace_back(&this->ff_gain);
    speedFromFeedForward();
    frequency.emplace_back(0.15);
    alpha.emplace_back(0.5);
    gamma.emplace_back(0.2);
    low_bound.emplace_back(0.00001);
    high_bound.emplace_back(100);
}

double FeedForward::evaluate(double error) {
    uFF = ff_gain * get_speed();
    double out = controller->evaluate(error);
    return out + uFF;
}

double FeedForward::simulate(double error) const {
    return controller->simulate(error) + ff_gain * get_speed();
}

double & FeedForward::getFeedForwardRef() {
    return ff_gain;
}

void FeedForward::serialize(JsonObject json) {
    json["type"] = this->type;
    SET_JSON(ff_gain);
    SET_JSON(feedforward_type);
    if(controller) {
        JsonObject nested = json["controller"].to<JsonObject>();
        controller->serialize(nested);
    }
}

std::shared_ptr<BasicController> FeedForward::deserialize(std::shared_ptr<BaseRobot> robot, JsonVariant &json) {
    return deserialize_as_T<FeedForward>(robot, json);
}

void FeedForward::speedFromFeedForward() {
    auto r = robot;
    if(!r) return;
    switch(feedforward_type) {
        case FeedForwardType::DISTANCE:
            get_speed = [r]() { return r->getTranslationalRampSpeed(); };
            break;
        case FeedForwardType::ANGLE:
            get_speed = [r]() { return r->getRotationalRampSpeed().toDegrees(); };
            break;
    }
}

std::vector<std::pair<double, double>> FeedForward::update_gains(std::vector<double> initialGain, double t) {
    auto a = BasicController::update_gains(initialGain, t);
    std::vector<double> subset(initialGain.begin() + a.size(), initialGain.end());
    auto b = this->controller->update_gains(subset, t);
    a.insert(a.end(), b.begin(), b.end());
    return a;
}

size_t FeedForward::final_update(std::vector<double> initialGain, std::vector<double> multiplier) {
    auto size = BasicController::final_update(initialGain, multiplier);
    initialGain = std::vector<double>(initialGain.begin() + size, initialGain.end());
    multiplier = std::vector<double>(multiplier.begin() + size, multiplier.end());
    size += this->controller->final_update(initialGain, multiplier);
    return size;
}

std::vector<double> FeedForward::getGains() {
    auto gains = BasicController::getGains();
    auto b = this->controller->getGains();
    gains.insert(gains.end(), b.begin(), b.end());
    return gains;
}

void FeedForward::setAlpha(double alpha) {
    BasicController::setAlpha(alpha);
    controller->setAlpha(alpha);
}

void FeedForward::setGamma(double gamma) {
    BasicController::setGamma(gamma);
    controller->setGamma(gamma);
}

void FeedForward::registerCommands(CommandParser &parser, const char* name) {
    if(controller) controller->registerCommands(parser, name);
    std::string command_name = std::string("ff_") + name;
    parser.registerMathCommand(command_name.c_str(), ff_gain, [name](Stream &s, double v, MathOP){
        s.printf("Feed forward %s is : %f\r\n", name, v);
        return "";
    }, "Change or view feed forward value");
}

void FeedForward::unregisterCommands(CommandParser &parser, const char* name) {
    if(controller) controller->unregisterCommands(parser, name);
    parser.removeAllCommands((std::string("ff_") + name).c_str());
}

void FeedForward::multiply(double d) {
    ff_gain *= d;
    if(controller) controller->multiply(d);
}

