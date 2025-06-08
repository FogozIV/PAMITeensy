//
// Created by fogoz on 24/04/2025.
//

#ifndef PAMITEENSY_SPEEDESTIMATOR_H
#define PAMITEENSY_SPEEDESTIMATOR_H
#include <memory>

#include "ConditionVariable.h"
//Based on PLLEstimator
//https://discourse.odriverobotics.com/t/rotor-encoder-pll-and-velocity/224/2
//https://github.com/EsialRobotik/asserv_chibios/blob/master/src/Pll.cpp


class BaseRobot;

/**
 * @brief Phase-Locked Loop based speed estimator
 * 
 * This class implements a PLL-based speed estimation algorithm based on:
 * - https://discourse.odriverobotics.com/t/rotor-encoder-pll-and-velocity/224/2
 * - https://github.com/EsialRobotik/asserv_chibios/blob/master/src/Pll.cpp
 * 
 * The estimator provides:
 * - Smooth speed estimation
 * - Position tracking
 * - Configurable bandwidth
 * - Noise filtering
 * 
 * The PLL approach offers advantages over simple differentiation:
 * - Better noise rejection
 * - Smoother velocity estimates
 * - Phase tracking capability
 */
class SpeedEstimator{
    double kp = 0;
    double ki = 0;
    double speed = 0.0f;
    double distance_estimation = 0.0f;
    double real_distance = 0.0f;
    std::shared_ptr<BaseRobot> baseRobot;
    ConditionalVariable bandwidth;
public:
    /**
     * @brief Constructs a new speed estimator
     * 
     * @param baseRobot Reference to robot instance
     * @param bandwidth Initial PLL bandwidth (Hz)
     */
    SpeedEstimator(std::shared_ptr<BaseRobot> baseRobot, double bandwidth);

    /**
     * @brief Updates the speed estimate
     * 
     * This method:
     * - Updates position tracking
     * - Computes new speed estimate
     * - Updates PLL state
     * 
     * @param distance New position measurement
     */
    void update(double distance);

    /**
     * @brief Resets the estimator state
     * 
     * Resets:
     * - Speed estimate
     * - Position tracking
     * - PLL integrator
     */
    void reset();

    /**
     * @brief Sets the PLL bandwidth
     * 
     * Higher bandwidth values result in:
     * - Faster response
     * - More noise sensitivity
     * Lower values provide:
     * - Better noise rejection
     * - Slower response
     * 
     * @param bandwidth New bandwidth value (Hz)
     */
    void setBandwidth(double bandwidth);

    /**
     * @brief Gets the current speed estimate
     * @return double Current speed
     */
    double getSpeed() const;

    /**
     * @brief Gets the estimated position
     * @return double Estimated position
     */
    double getDistanceEstimation() const;

    /**
     * @brief Gets the actual measured position
     * @return double Real position
     */
    double getRealDistance() const;

    /**
     * @brief Gets the current PLL bandwidth
     * @return double Current bandwidth (Hz)
     */
    double getBandwidth() const;

    /**
     * @brief Gets reference to bandwidth parameter
     * @return ConditionalVariable& Bandwidth parameter reference
     */
    ConditionalVariable& getBandwidthRef();

};


#endif //PAMITEENSY_SPEEDESTIMATOR_H
