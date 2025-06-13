//
// Created by fogoz on 13/06/2025.
//

#ifndef PIDFILTEREDD_H
#define PIDFILTEREDD_H

#include <functional>
#include <memory>
#include "BasicController.h"
#include "PID.h"
class BaseRobot;
/**
 * @brief PID (Proportional-Integral-Derivative) controller implementation
 *
 * This class implements a PID controller for closed-loop control systems.
 * It provides functionality for tuning control parameters and includes
 * anti-windup protection to prevent integral term saturation.
 */
class PIDFilteredD : public PID {
protected:
    double N = 10;  ///< Filter variable
    double rawUd = 0; ///< The raw value of Ud without filtering
public:
    /**
     * @brief Constructs a new PID controller
     *
     * @param robot Reference to the robot being controlled
     * @param kp Proportional gain coefficient
     * @param ki Integral gain coefficient
     * @param kd Derivative gain coefficient
     * @param anti_windup Maximum value for integral term to prevent windup
     * @param N the value that we use to tune the filter
     */
    PIDFilteredD(std::shared_ptr<BaseRobot> robot, double kp, double ki, double kd, double anti_windup, double N=10);

    /**
     *@brief Construct an empty PID controller
     *
     *@param robot Reference to the robot being controllerd
     *@param pid a pid to build the PIDFilteredD from
     */
    PIDFilteredD(std::shared_ptr<BaseRobot> robot, const std::shared_ptr<PID>& pid=nullptr);

    /**
     * @brief Evaluates the PID control output for a given error
     *
     * @param error Current error value (setpoint - measured_value)
     * @return double Control output value
     */
    double evaluate(double error) override;

    double simulate(double error) const override;

    double& getNRef();

    /**
     * @brief Serializes PID controller parameters to JSON
     *
     * @param json JSON object to store parameters
     */
    void serialize(JsonObject json) override;

    [[nodiscard]] double getRawUd() const;

    /**
     * @brief Creates a PID controller from JSON configuration
     *
     * @param robot Reference to the robot being controlled
     * @param json JSON object containing PID parameters
     * @return std::shared_ptr<BasicController> Configured PID controller
     */
    std::shared_ptr<BasicController> deserialize(std::shared_ptr<BaseRobot> robot, JsonVariant &json) override;
    /**
     * @brief Creates a PID controller from JSON configuration
     *
     * @param robot Reference to the robot being controlled
     * @param json JSON object containing PID parameters
     * @return std::shared_ptr<BasicController> Configured PID controller
     */
    template<typename T>
    static std::shared_ptr<T> deserialize_as_T(std::shared_ptr<BaseRobot> robot, const JsonVariant &json);

    void registerCommands(CommandParser &parser, const char* name) override;

    void unregisterCommands(CommandParser &parser, const char* name) override;

    void reset(double error) override;
};

template<typename T>
std::shared_ptr<T> PIDFilteredD::deserialize_as_T(std::shared_ptr<BaseRobot> robot, const JsonVariant &json) {
    std::shared_ptr<T> p = std::make_shared<T>(robot);
    GET_AND_CHECK_JSON(p, ki, double);
    GET_AND_CHECK_JSON(p, kd, double);
    GET_AND_CHECK_JSON(p, kp, double);
    GET_AND_CHECK_JSON(p, anti_windup, double);
    GET_AND_CHECK_JSON(p, N, double);
    return p;
}

#endif //PIDFILTEREDD_H
