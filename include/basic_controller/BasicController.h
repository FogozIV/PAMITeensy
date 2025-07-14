//
// Created by fogoz on 24/04/2025.
//

#ifndef PAMITEENSY_BASICCONTROLLER_H
#define PAMITEENSY_BASICCONTROLLER_H
#include "ArduinoJson.h"
#include <memory>
#include <CommandParser.h>
#include <utils/StreamSplitter.h>
#include <TeensyThreads.h>
namespace BasicControllerType {
    enum ControllerType {
        BasicController,
        PID,
        PIDSpeedFeedForward,
        PIDFilteredD,
        FeedForward
    };
}
class BaseRobot;
#define GET_AND_CHECK_JSON(var_name, name, type) \
    if(json[#name].is<type>()){\
        var_name->name = json[#name].as<type>(); \
    }

#define SET_JSON(name) json[#name] = this->name;
/**
 * @brief Base class for single-input controllers
 * 
 * This class defines the interface for basic feedback controllers
 * that operate on a single error value. It's used for:
 * - PID control
 * - Simple feedback loops
 * - Error-based control
 * 
 * The controller takes an error value (difference between
 * desired and actual values) and produces a control output.
 */
class BasicController {
protected:
    BasicControllerType::ControllerType type = BasicControllerType::BasicController;
    std::vector<double*> variables = {};
    //frequency, alpha, gamma
public:
    std::vector<double> frequency = {};
    std::vector<double> alpha = {};
    std::vector<double> gamma = {};
    std::vector<double> low_bound = {};
    std::vector<double> high_bound = {};
    /**
     * @brief Evaluates controller output
     * 
     * This method computes the control output based on
     * the current error value. The error is typically:
     * error = setpoint - measured_value
     * 
     * @param error Current error value
     * @return double Control output
     */
    virtual double evaluate(double error) = 0;

    /**
     * @brief Resets controller with initial error
     * 
     * Resets the controller state using a specific
     * initial error value. This is useful for:
     * - Initialization
     * - Mode changes
     * - Preventing control bumps
     * 
     * @param error Initial error value
     */
    virtual void reset(double error) = 0;

    /**
     * @brief Evaluate control output without changing the controller state
     * @param error The current error
     * @return double Theoretical control output
     */
    virtual double simulate(double error) const = 0;

    /**
     * @brief Resets controller to zero
     * 
     * Convenience method that resets the controller
     * with an initial error of zero.
     */
    void reset() {
        reset(0.0f);
    }

    /**
     * @brief Virtual destructor
     */
    virtual ~BasicController() = default;

    virtual BasicControllerType::ControllerType getType() {
        return type;
    }

    virtual void serialize(JsonObject json) = 0;

    virtual std::shared_ptr<BasicController> deserialize(std::shared_ptr<BaseRobot> robot, JsonVariant& json)  = 0;

    virtual void registerCommands(CommandParser& parser, const char* name) = 0;


    virtual void unregisterCommands(CommandParser& parser, const char* name) = 0;

    inline virtual std::vector<std::pair<double, double>> update_gains(std::vector<double> initialGain, double t) {
        std::vector<std::pair<double, double>> gains;
        for (size_t i = 0; i < variables.size(); i++) {
            *variables[i] = initialGain[i] + alpha[i] * initialGain[i] * cos(2*PI*t*frequency[i]);
            gains.emplace_back(cos(t*frequency[i]), sin(t*frequency[i]));
        }
        return gains;
    }

    inline virtual size_t final_update(std::vector<double> initialGain, std::vector<double> multiplier) {
        for (size_t i = 0; i < variables.size(); i++) {
            *variables[i] = initialGain[i] - gamma[i] * multiplier[i];
            *variables[i] = constrain(*variables[i], low_bound[i], high_bound[i]);
            streamSplitter.printf("Variable %d is updated to %f\r\n", i, *variables[i]);
        }
        return variables.size();
    }

    virtual std::vector<double> getGains() {
        std::vector<double> gains;
        for (size_t i = 0; i < variables.size(); i++) {
            gains.emplace_back(*variables[i]);
        }
        return gains;
    }

    /**
     * This function allows to multiply the core values of the controller
     * @param d the constant by which we multiply all parameters
     */
    virtual void multiply(double d) = 0;
};

#endif //PAMITEENSY_BASICCONTROLLER_H
