//
// Created by fogoz on 10/05/2025.
//

#ifndef CURVETARGET_H
#define CURVETARGET_H
#include "BaseTarget.h"
#include "curves/BaseCurve.h"
#include "ramp/Ramp.h"

/**
 * @brief Curve following motion target
 * 
 * This class implements a motion target that follows a
 * parametric curve with speed control. Features:
 * - Curve path following
 * - Speed ramping
 * - Position tracking
 * - Completion detection
 * 
 * The target uses:
 * - Parametric curve for path definition
 * - Speed ramping for smooth motion
 * - Position feedback for tracking
 * 
 * @tparam Ramp Speed ramping implementation type
 */
template<typename Ramp>
class CurveTarget : public BaseTarget {
    bool done = false;                    ///< Completion flag
    RampData rampData;                    ///< Speed ramping parameters
    std::shared_ptr<BaseCurve> curve;     ///< Path curve
    std::shared_ptr<Ramp> ramp;           ///< Speed ramping
    double t;                             ///< Curve parameter
    double step;                          ///< Path step size
    Position target_pos;                  ///< Target position

public:
    /**
     * @brief Constructs a new curve target
     * 
     * @param robot Robot instance
     * @param curve Path curve to follow
     * @param ramp Speed ramping parameters
     * @param step Path step size (default: 20.0)
     */
    explicit CurveTarget(const std::shared_ptr<BaseRobot> &robot, 
                        std::shared_ptr<BaseCurve> curve, RampData ramp, 
                        double step=20.0);

    /**
     * @brief Checks if target is complete
     * @return bool True if curve following is complete
     */
    bool is_done() override;

    /**
     * @brief Initializes curve following
     * 
     * Sets up:
     * - Initial position
     * - Speed ramping
     * - Path tracking
     */
    void init() override;

    /**
     * @brief Handles target completion
     * 
     * Performs cleanup and notifies completion
     */
    void on_done() override;

    /**
     * @brief Processes curve following
     * 
     * Updates:
     * - Current position
     * - Speed control
     * - Path progress
     */
    void process() override;

    /**
     * @brief Reinitializes after stop
     * 
     * Restores state for:
     * - Position tracking
     * - Speed control
     * - Path following
     */
    void reInitAfterStop() override;
};

#include "CurveTarget.tpp"

#endif //CURVETARGET_H
