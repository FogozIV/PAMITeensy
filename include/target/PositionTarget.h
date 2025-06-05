//
// Created by fogoz on 07/05/2025.
//

#ifndef POSITIONTARGET_H
#define POSITIONTARGET_H
#include "BaseTarget.h"
#include "ramp/Ramp.h"
#include "robot/BaseRobot.h"

#define MAKE_POSITION_TARGET(pos, ramp_data, ramp_type) std::make_shared<PositionTarget<ramp_type>>(robot, pos, ramp_data)

#define DEFAULT_MAKE_POSITION_TARGET(pos, ramp_data) MAKE_POSITION_TARGET(pos, ramp_data, CalculatedQuadramp)

#define COMPLETE_POSITION_TARGET(pos, ramp_data) robot->addTarget(DEFAULT_MAKE_POSITION_TARGET(pos, ramp_data))
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
class PositionTarget : public BaseTarget {
protected:
    Position pos;                             ///< Target position
    std::shared_ptr<Ramp> ramp = nullptr;     ///< Speed ramping
    RampData ramp_data;                       ///< Ramping parameters
    std::function<double()> distanceComputer;  ///< Distance calculator
    double previous_trans_pos; ///<Previous curvilinear pos

public:
    /**
     * @brief Constructs a new position target
     * 
     * @param baseRobot Robot instance
     * @param pos Target position
     * @param rampData Speed ramping parameters
     */
    PositionTarget(std::shared_ptr<BaseRobot> baseRobot, Position pos, 
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

#include "PositionTarget.tpp"

#endif //POSITIONTARGET_H
