//
// Created by fogoz on 03/05/2025.
//

#ifndef PAMITEENSY_BASICQUADRAMP_H
#define PAMITEENSY_BASICQUADRAMP_H

#include "Ramp.h"
#include <memory>
#include <functional>

class BaseRobot;

/**
 * @brief Basic quadratic motion profile generator
 * 
 * This class implements a simplified quadratic motion profile with:
 * - Single acceleration/deceleration rate
 * - Speed limiting
 * - Position tracking
 * - Simple profile calculation
 * 
 * The profile provides:
 * - Smooth speed transitions
 * - Basic motion control
 * - Position-based speed adjustment
 * - Minimal parameter set
 */
class BasicQuadRamp : public Ramp {
    std::shared_ptr<BaseRobot> robot;              ///< Robot reference
    double acc;                                    ///< Acceleration/deceleration rate
    double maxSpeed;                               ///< Maximum speed limit

    double endSpeed;                               ///< Target end speed
    double currentSpeed = 0;                       ///< Current speed
    std::function<double()> distanceToPoint;       ///< Distance callback

public:
    /**
     * @brief Constructs from ramp data
     * 
     * @param robot Robot instance
     * @param data Ramping parameters
     * @param distanceToPoint Distance measurement callback
     */
    BasicQuadRamp(std::shared_ptr<BaseRobot> robot, RampData data, 
                  const std::function<double()> &distanceToPoint);

    /**
     * @brief Starts the ramping process
     * @param initialSpeed Starting speed
     */
    void start(double initialSpeed) override;

    /**
     * @brief Computes speed change
     * 
     * Calculates speed change using:
     * - Current speed
     * - Acceleration rate
     * - Speed limits
     * - Position feedback
     * 
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
     * 
     * Initiates deceleration to stop using
     * the configured acceleration rate.
     */
    void stop() override;
};

#endif //PAMITEENSY_BASICQUADRAMP_H
