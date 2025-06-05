//
// Created by fogoz on 08/05/2025.
//

#ifndef ANGLETARGET_H
#define ANGLETARGET_H
#include "BaseTarget.h"
#include "ramp/Ramp.h"
#include "utils/Angle.h"

#define MAKE_ANGLE_TARGET(angle, rampdata, ramp_type) \
    std::make_shared<AngleTarget<ramp_type>>(robot, angle, rampdata)

#define DEFAULT_MAKE_ANGLE_TARGET(angle, rampdata) MAKE_ANGLE_TARGET(angle, rampdata, CalculatedQuadramp)

#define COMPLETE_ANGLE_TARGET(angle, rampdata) DEFAULT_MAKE_ANGLE_TARGET(angle, rampdata)

/**
 * @brief Angular motion target
 * 
 * This class implements a motion target that rotates to a
 * specified angle with speed control. Features:
 * - Angle tracking
 * - Speed ramping
 * - Rotation control
 * - Completion detection
 * 
 * The target uses:
 * - Angular feedback
 * - Speed ramping for smooth rotation
 * - Angle normalization
 * 
 * @tparam T Ramping implementation type
 */
template<typename T>
class AngleTarget : public BaseTarget {
protected:
    std::shared_ptr<Ramp> ramp;       ///< Speed ramping
    RampData ramp_data;               ///< Ramping parameters
    Angle target_angle;               ///< Target angle

public:
    /**
     * @brief Constructs a new angle target
     * 
     * @param robot Robot instance
     * @param target_angle Target rotation angle
     * @param rampData Speed ramping parameters
     */
    explicit AngleTarget(const std::shared_ptr<BaseRobot> &robot, 
                        Angle target_angle, RampData rampData)
        : BaseTarget(robot), ramp_data(rampData), target_angle(target_angle) {
    }

    /**
     * @brief Handles target completion
     * 
     * Performs cleanup and notifies completion
     */
    void on_done() override;

    /**
     * @brief Checks if target is complete
     * @return bool True if angle is reached
     */
    bool is_done() override;

    /**
     * @brief Initializes angle tracking
     * 
     * Sets up:
     * - Initial angle
     * - Speed ramping
     * - Rotation control
     */
    void init() override;

    /**
     * @brief Processes angle control
     * 
     * Updates:
     * - Current angle
     * - Speed control
     * - Rotation tracking
     */
    void process() override;

    /**
     * @brief Reinitializes after stop
     * 
     * Restores state for:
     * - Angle tracking
     * - Speed control
     * - Rotation control
     */
    void reInitAfterStop() override;
};

#include "AngleTarget.tpp"

#endif //ANGLETARGET_H

//
// Created by fogoz on 08/05/2025.
//

#ifndef ANGLETARGET_H
#define ANGLETARGET_H
#include "BaseTarget.h"
#include "ramp/Ramp.h"
#include "utils/Angle.h"


template<typename T>
class AngleTarget : public BaseTarget{
    bool done = false;
    std::shared_ptr<Ramp> ramp;
    RampData ramp_data;
    Angle target_angle;
public:
    explicit AngleTarget(const std::shared_ptr<BaseRobot> &robot, Angle target_angle, RampData rampData)
        : BaseTarget(robot), ramp_data(rampData), target_angle(target_angle) {
    }

    void on_done() override;

    bool is_done() override;

    void init() override;

    void process() override;

    void reInitAfterStop() override;
};

#include "AngleTarget.tpp"



#endif //ANGLETARGET_H
