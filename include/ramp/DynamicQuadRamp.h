//
// Created by fogoz on 27/04/2025.
//

#ifndef DYNAMICQUADRAMP_H
#define DYNAMICQUADRAMP_H

#include "Ramp.h"
#include <functional>
#include "../robot/BaseRobot.h"
#include <memory>

/**
 * @brief Dynamic quadratic motion profile generator
 * 
 * This class implements a quadratic motion profile with:
 * - Independent acceleration/deceleration rates
 * - Dynamic speed adjustment
 * - Position-based control
 * - Real-time profile updates
 * 
 * The profile dynamically adjusts based on:
 * - Current position
 * - Target position
 * - Speed limits
 * - Acceleration constraints
 */
class DynamicQuadRamp : public Ramp {
    std::shared_ptr<BaseRobot> robot;              ///< Robot reference
    RampData ramp;
    double currentSpeed{};                           ///< Current speed
    std::function<double()> distanceToPoint;       ///< Distance callback
    std::function<double()> curvature;

public:
    /**
     * @brief Constructs a new dynamic ramp
     * 
     * @param robot Robot instance
     * @param ramp All the RampData such has acceleration, maxSpeed, deceleration
     * @param distanceToPoint Distance measurement callback
     * @param curvature the curvature of the target
     */
    DynamicQuadRamp(std::shared_ptr<BaseRobot> robot, RampData ramp, std::function<double()> distanceToPoint, std::function<double()> curvature=[](){return 0.0;});

    /**
     * @brief Starts the ramping process
     * @param initialSpeed Starting speed
     */
    void start(double initialSpeed) override;

    /**
     * @brief Computes speed change
     * 
     * Dynamically calculates speed change based on:
     * - Current position
     * - Remaining distance
     * - Speed limits
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
     * the configured deceleration rate.
     */
    void stop() override;
};

#endif //DYNAMICQUADRAMP_H
