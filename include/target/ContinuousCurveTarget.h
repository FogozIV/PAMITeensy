//
// Created by fogoz on 12/06/2025.
//

#ifndef PAMITEENSY_CONTINUOUSCURVETARGET_H
#define PAMITEENSY_CONTINUOUSCURVETARGET_H
#include "BaseTarget.h"
#include "curves/BaseCurve.h"
#include "ramp/Ramp.h"
#include <robot/RobotTolerance.h>

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
class ContinuousCurveTarget : public BaseTarget {
    RampData rampData;                    ///< Speed ramping parameters
    std::shared_ptr<BaseCurve> curve;     ///< Path curve
    std::shared_ptr<Ramp> ramp;           ///< Speed ramping
    double ahead_distance;                ///< Path step size
    Position target_pos;                  ///< Target position
    double t;                             ///< Last parameter value
    double startingCurvilinearDistance;   ///< The value of the starting distance
    Position final_pos;                   ///< The value of the final pos
    uint16_t tick;                        ///< The tick to end everything
    uint16_t distance_recalculator = 1;
public:
    /**
     * @brief Constructs a new curve target
     *
     * @param robot Robot instance
     * @param curve Path curve to follow
     * @param ramp Speed ramping parameters
     * @param ahead_distance the angular target point position
     */
    explicit ContinuousCurveTarget(const std::shared_ptr<BaseRobot> &robot,
                         std::shared_ptr<BaseCurve> curve, RampData ramp, double ahead_distance=10);

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

    Position getTargetPosition();
};

#include "ContinuousCurveTarget.tpp"

#endif //PAMITEENSY_CONTINUOUSCURVETARGET_H
