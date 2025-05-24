//
// Created by fogoz on 24/04/2025.
//

#ifndef PAMITEENSY_POSITIONMANAGER_H
#define PAMITEENSY_POSITIONMANAGER_H

#include <memory>
#include <TeensyThreads.h>

#include "ArduinoJson.h"

#include "encoders/BaseEncoder.h"
#include "utils/SpeedEstimator.h"
#include "utils/Position.h"

/**
 * @brief Configuration parameters for position tracking
 * 
 * This structure holds the physical parameters needed for
 * odometry calculations:
 * - Track width (distance between wheels)
 * - Wheel diameters (which may differ between sides)
 */
struct PositionParameters{
    double track_mm = 10;           ///< Distance between wheels in mm
    double left_wheel_diam = 1;     ///< Left wheel diameter in mm
    double right_wheel_diam = 1;    ///< Right wheel diameter in mm

    /**
     * @brief Scales all dimensional parameters
     * @param factor Scale factor to apply
     */
    void multiply(double factor) {
        left_wheel_diam *= factor;
        right_wheel_diam *= factor;
    }

    /**
     * @brief Constructs position parameters
     * @param track_mm Track width in mm
     * @param left_wheel_diam Left wheel diameter in mm
     * @param right_wheel_diam Right wheel diameter in mm
     */
    PositionParameters(double track_mm=10, double left_wheel_diam = 1, double right_wheel_diam = 1) : track_mm(track_mm), left_wheel_diam(left_wheel_diam), right_wheel_diam(right_wheel_diam) {};
};

class BaseRobot;

/**
 * @brief Manages robot position tracking
 * 
 * This class handles odometry calculations to track the robot's position.
 * It uses:
 * - Wheel encoders for motion measurement
 * - Speed estimators for velocity tracking
 * - Thread-safe position updates
 * 
 * The position manager:
 * - Tracks absolute position (x,y,θ)
 * - Computes incremental motion
 * - Estimates linear and angular velocity
 * - Handles wheel diameter calibration
 */
class PositionManager {
    std::shared_ptr<BaseRobot> robot;                ///< Reference to robot instance
    std::shared_ptr<BaseEncoder> leftWheelEncoder;   ///< Left wheel encoder
    std::shared_ptr<BaseEncoder> rightWheelEncoder;  ///< Right wheel encoder
    std::shared_ptr<PositionParameters> params;      ///< Physical parameters
    Position deltaPos;                               ///< Incremental position change
    double deltaDistance = 0.0;                      ///< Incremental distance moved
    double deltaAngle = 0.0;                         ///< Incremental angle change
    mutable std::mutex mutex;                        ///< Thread safety mutex
    std::shared_ptr<SpeedEstimator> distanceEstimator;  ///< Linear speed estimator
    std::shared_ptr<SpeedEstimator> angleEstimator;     ///< Angular speed estimator

public:
    /**
     * @brief Constructs a new position manager
     * 
     * @param robot Reference to robot instance
     * @param leftWheelEncoder Left wheel encoder
     * @param rightWheelEncoder Right wheel encoder
     * @param params Physical parameters
     * @param distanceEstimator Optional linear speed estimator
     * @param angleEstimator Optional angular speed estimator
     */
    PositionManager(const std::shared_ptr<BaseRobot> &robot, const std::shared_ptr<BaseEncoder> &leftWheelEncoder,
                    const std::shared_ptr<BaseEncoder> &rightWheelEncoder, const std::shared_ptr<PositionParameters> &params, const std::shared_ptr<SpeedEstimator> &distanceEstimator = nullptr, const std::shared_ptr<SpeedEstimator>&angleEstimator = nullptr);

    /**
     * @brief Updates position tracking
     * 
     * Computes new position based on:
     * - Encoder readings
     * - Physical parameters
     * - Previous position
     * 
     * @param pos Current position
     * @return tuple<Position,double,double> New position, distance moved, angle change
     */
    std::tuple<Position, double, double> computePosition(const Position &pos);

    /**
     * @brief Gets incremental position change
     * @return Position Recent position delta
     */
    Position getDeltaPos() const;

    /**
     * @brief Gets incremental distance moved
     * @return double Recent distance delta
     */
    double getDeltaDist() const;

    /**
     * @brief Gets incremental angle change
     * @return Angle Recent angle delta
     */
    Angle getDeltaAngle() const;

    /**
     * @brief Updates encoders and parameters
     * 
     * Allows changing:
     * - Encoder instances
     * - Physical parameters
     * Useful for:
     * - Hardware reconfiguration
     * - Calibration updates
     * 
     * @param leftEncoder New left encoder
     * @param rightEncoder New right encoder
     * @param params New physical parameters
     */
    void overrideLeftRightEncoder(std::shared_ptr<BaseEncoder> leftEncoder, std::shared_ptr<BaseEncoder> rightEncoder, std::shared_ptr<PositionParameters> params);
};

/**
 * @brief JSON converter for position parameters
 * 
 * Provides serialization and deserialization of position
 * parameters to/from JSON format.
 */
ARDUINOJSON_BEGIN_PUBLIC_NAMESPACE
template <>
struct Converter<PositionParameters> {
    /**
     * @brief Converts parameters to JSON
     * @param src Source parameters
     * @param dst Destination JSON object
     */
    static void toJson(const PositionParameters& src, JsonVariant dst) {
        dst["track_mm"] = src.track_mm;
        dst["left_wheel_diam"] = src.left_wheel_diam;
        dst["right_wheel_diam"] = src.right_wheel_diam;
    }

    /**
     * @brief Creates parameters from JSON
     * @param src Source JSON object
     * @return PositionParameters Constructed parameters
     */
    static PositionParameters fromJson(JsonVariantConst src) {
        return {src["track_mm"].as<double>(), src["left_wheel_diam"], src["right_wheel_diam"]};
    }

    /**
     * @brief Validates JSON format
     * @param src JSON object to check
     * @return bool True if JSON is valid
     */
    static bool checkJson(JsonVariantConst src) {
        return src["track_mm"].is<double>() && src["left_wheel_diam"].is<double>() && src["right_wheel_diam"].is<double>();
    }
};


ARDUINOJSON_END_PUBLIC_NAMESPACE


#endif //PAMITEENSY_POSITIONMANAGER_H
