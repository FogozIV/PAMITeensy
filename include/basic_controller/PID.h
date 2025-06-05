//
// Created by fogoz on 24/04/2025.
//

#ifndef PID_H
#define PID_H

#include <functional>
#include <memory>
#include "BasicController.h"
#include "ArduinoJson.h"

class BaseRobot;

/**
 * @brief PID (Proportional-Integral-Derivative) controller implementation
 * 
 * This class implements a PID controller for closed-loop control systems.
 * It provides functionality for tuning control parameters and includes
 * anti-windup protection to prevent integral term saturation.
 */
class PID : public BasicController {
    std::shared_ptr<BaseRobot> robot;
    double kp;  ///< Proportional gain
    double ki;  ///< Integral gain
    double kd;  ///< Derivative gain

    double old_error = 0;  ///< Previous error value for derivative calculation
    double iTerm = 0;      ///< Accumulated integral term
    double anti_windup;    ///< Anti-windup limit for integral term
public:
    /**
     * @brief Constructs a new PID controller
     * 
     * @param robot Reference to the robot being controlled
     * @param kp Proportional gain coefficient
     * @param ki Integral gain coefficient
     * @param kd Derivative gain coefficient
     * @param anti_windup Maximum value for integral term to prevent windup
     */
    PID(std::shared_ptr<BaseRobot> robot, double kp, double ki, double kd, double anti_windup);

    /**
     * @brief Evaluates the PID control output for a given error
     * 
     * @param error Current error value (setpoint - measured_value)
     * @return double Control output value
     */
    double evaluate(double error) override;



    /**
     * @brief Sets the proportional gain
     * @param kp New proportional gain value
     */
    void setKP(double kp);

    /**
     * @brief Sets the integral gain
     * @param ki New integral gain value
     */
    void setKI(double ki);

    /**
     * @brief Sets the derivative gain
     * @param kd New derivative gain value
     */
    void setKD(double kd);

    /**
     * @brief Sets the anti-windup limit
     * @param anti_windup New anti-windup limit value
     */
    void setAntiWindup(double anti_windup);

    /**
     * @brief Resets the controller state
     * 
     * Resets integral term and previous error value
     * @param error Initial error value
     */
    void reset(double error) override;

    /**
     * @brief Gets the current derivative gain
     * @return Current Kd value
     */
    double getKd() const;

    /**
     * @brief Gets the current integral gain
     * @return Current Ki value
     */
    double getKi() const;

    /**
     * @brief Gets the current proportional gain
     * @return Current Kp value
     */
    double getKp() const;

    /**
     * @brief Gets a reference to the derivative gain
     * @return Reference to Kd value
     */
    double& getKdRef();

    /**
     * @brief Gets a reference to the integral gain
     * @return Reference to Ki value
     */
    double& getKiRef();

    /**
     * @brief Gets a reference to the proportional gain
     * @return Reference to Kp value
     */
    double& getKpRef();

    /**
     * @brief Gets a reference to the anti-windup limit
     * @return Reference to anti-windup value
     */
    double& getAntiWindupRef();

    /**
     * @brief Gets the current anti-windup limit
     * @return Current anti-windup value
     */
    double getAntiWindup() const;

    /**
     * @brief Evaluate control output without changing iTerm and previous error
     * @param error The current error
     * @return double Theoretical control output
     */
    double simulate(double error) const override;
};

/**
 * @brief Creates a PID controller from JSON configuration
 * 
 * @param robot Reference to the robot being controlled
 * @param src JSON object containing PID parameters
 * @return std::shared_ptr<PID> Configured PID controller
 */
inline std::shared_ptr<PID> getPIDFromJson(std::shared_ptr<BaseRobot> robot, JsonVariantConst src){
    double kp;
    double ki;
    double kd;
    double anti_windup;
    if(src["kp"].is<double>()){
        kp = src["kp"].as<double>();
    }
    if(src["ki"].is<double>()){
        ki = src["ki"].as<double>();
    }
    if(src["kd"].is<double>()){
        kd = src["kd"].as<double>();
    }
    if(src["anti_windup"].is<double>()){
        anti_windup = src["anti_windup"].as<double>();
    }
    return std::make_shared<PID>(robot, kp, ki, kd, anti_windup);
}

/**
 * @brief Serializes PID controller parameters to JSON
 * 
 * @param pid PID controller to serialize
 * @param dst JSON object to store parameters
 */
inline void PIDtoJson(std::shared_ptr<PID> pid, JsonVariant dst){
    JsonObject variant = dst.to<JsonObject>();
    variant["kp"] = pid->getKp();
    variant["ki"] = pid->getKi();
    variant["kd"] = pid->getKd();
    variant["anti_windup"] = pid->getAntiWindup();
}

#endif //PID_H
