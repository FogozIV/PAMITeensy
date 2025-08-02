//
// Created by fogoz on 07/05/2025.
//

#ifndef DISTANCETARGET_H
#define DISTANCETARGET_H
#include "BaseTarget.h"
#include "ramp/Ramp.h"
#include "robot/BaseRobot.h"

#define MAKE_DISTANCE_TARGET(distance, rampdata, ramp_type) std::make_shared<DistanceTarget<ramp_type>>(robot, distance, rampdata)

#define DEFAULT_MAKE_DISTANCE_TARGET(distance, rampdata) MAKE_DISTANCE_TARGET(distance, rampdata, CalculatedQuadramp)

#define COMPLETE_DISTANCE_TARGET(distance, rampdata) robot->addTarget(DEFAULT_MAKE_DISTANCE_TARGET(distance, rampdata))

#define STEP_DISTANCE_TARGET(distance) robot->addTarget(MAKE_DISTANCE_TARGET(distance, RampData(), Step))

/**
 * @brief Position-based motion target
 *
 * This class implements a motion target that moves to a
 * specified position with speed control. Features:
 * - Position tracking
 * - Speed ramping
 * - Distance computation
 * - Completion detection
 *
 * The target uses:
 * - Position feedback
 * - Speed ramping for smooth motion
 * - Distance-based control
 *
 * @tparam T Ramping implementation type
 */
template<typename T>
class DistanceTarget : public BaseTarget {
protected:
    double distance;                             ///< Target position
    std::shared_ptr<Ramp> ramp = nullptr;     ///< Speed ramping
    RampData ramp_data;                       ///< Ramping parameters
    std::function<double()> distanceComputer;  ///< Distance calculator
    double previous_trans_pos; ///<Previous curvilinear pos
    uint16_t done_tick; ///<Done tick for stall detection

public:
    /**
     * @brief Constructs a new position target
     *
     * @param baseRobot Robot instance
     * @param pos Target position
     * @param rampData Speed ramping parameters
     */
    DistanceTarget(std::shared_ptr<BaseRobot> baseRobot, double distance,
                  RampData rampData);

    /**
     * @brief Checks if target is complete
     * @return bool True if position is reached
     */
    bool is_done() override;

    /**
     * @brief Initializes position tracking
     *
     * Sets up:
     * - Initial position
     * - Speed ramping
     * - Distance computation
     */
    void init() override;

    /**
     * @brief Processes position control
     *
     * Updates:
     * - Current position
     * - Speed control
     * - Distance tracking
     */
    void process() override;

    /**
     * @brief Handles target completion
     *
     * Performs cleanup and notifies completion
     */
    void on_done() override;

    /**
     * @brief Reinitializes after stop
     *
     * Restores state for:
     * - Position tracking
     * - Speed control
     * - Distance computation
     */
    void reInitAfterStop() override;

};

#include "DistanceTarget.tpp"

#endif //DISTANCETARGET_H
