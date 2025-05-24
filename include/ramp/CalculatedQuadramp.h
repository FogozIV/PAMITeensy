//
// Created by fogoz on 08/05/2025.
//

#ifndef CALCULATEDQUADRAMP_H
#define CALCULATEDQUADRAMP_H
#include <memory>

#include "Ramp.h"
#include "robot/BaseRobot.h"

/**
 * @brief Calculated quadratic ramp parameters
 * 
 * This structure holds the calculated parameters for a
 * quadratic motion profile, including:
 * - Speed parameters
 * - Time intervals
 * - Distance segments
 * - Initial conditions
 */
struct CalculatedQuadrampData {
    double initial_speed;    ///< Starting speed
    double end_speed;       ///< Target end speed
    double acc_time;        ///< Acceleration phase duration
    double dec_time;        ///< Deceleration phase duration
    double acc_distance;    ///< Distance covered during acceleration
    double dec_distance;    ///< Distance covered during deceleration
    double ste_speed;       ///< Steady-state speed
    double ste_time;        ///< Steady-state phase duration

    bool inversed;          ///< Direction flag
    double initial_distance;  ///< Starting position
};

/**
 * @brief Quadratic motion profile generator
 * 
 * This class implements a quadratic motion profile that provides:
 * - Smooth acceleration and deceleration
 * - Position-based speed control
 * - Real-time profile calculation
 * - Distance tracking
 * 
 * The profile consists of three phases:
 * 1. Acceleration: Quadratic speed increase
 * 2. Steady-state: Constant speed
 * 3. Deceleration: Quadratic speed decrease
 */
class CalculatedQuadramp : public Ramp {
    std::shared_ptr<BaseRobot> robot;              ///< Robot reference
    RampData data;                                 ///< Ramping parameters
    std::function<double()> distanceToPoint;       ///< Distance callback
    CalculatedQuadrampData calculatedData;         ///< Calculated profile
    double previous_value = 0;                     ///< Previous position
    double t = 0;                                  ///< Current time
    double current_speed = 0;                      ///< Current speed

public:
    /**
     * @brief Constructs a new quadratic ramp
     * 
     * @param robot Robot instance
     * @param data Ramping parameters
     * @param distanceToPoint Distance measurement callback
     */
    CalculatedQuadramp(std::shared_ptr<BaseRobot> robot, RampData data, 
                       std::function<double()> distanceToPoint);

    /**
     * @brief Computes profile at specified time
     * 
     * @param t Time point
     * @return double Profile value at time t
     */
    double computeAtTime(double t);

    /**
     * @brief Starts the ramping process
     * @param initialSpeed Starting speed
     */
    void start(double initialSpeed) override;

    /**
     * @brief Computes speed change
     * @return double Speed delta
     */
    double computeDelta() override;

    /**
     * @brief Gets current speed
     * @return double Current speed
     */
    double getCurrentSpeed() override;

    /**
     * @brief Stops the ramping process
     */
    void stop() override;

    /**
     * @brief Virtual destructor
     */
    ~CalculatedQuadramp() override;
};

#endif //CALCULATEDQUADRAMP_H
