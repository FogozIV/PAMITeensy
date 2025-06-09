//
// Created by fogoz on 24/04/2025.
//

#ifndef PAMITEENSY_SIMPLETRIPLEBASICCONTROLLER_H
#define PAMITEENSY_SIMPLETRIPLEBASICCONTROLLER_H

#include "ArduinoJson.h"

#include "BaseController.h"
#include "memory"
#include "basic_controller/BasicController.h"
#include "utils/Angle.h"

class BaseRobot;

namespace TripleController {
    enum SpeedMode {
        NOT_SPEED,
        SPEED_DEFAULT
    };
}

/**
 * @brief Parameters for triple controller configuration
 * 
 * This structure holds configuration parameters for the three-controller
 * system that manages robot motion:
 * - Linear motion (distance)
 * - Angular motion (rotation)
 * - Combined motion (distance-angle coordination)
 */
struct TripleBasicParameters{
    double speed_min_l = 0;
    double speed_min_r = 0;

    double maxValueDistance = 4095;
    double maxValueDistanceAngle = 4095;
    double maxValueAngular = 4095;
    TripleController::SpeedMode speed_mode = TripleController::NOT_SPEED;
};

/**
 * @brief Triple controller implementation for robot motion
 * 
 * This class implements a control system using three controllers:
 * - Distance Controller: Manages linear motion
 * - Angle Controller: Manages rotational motion
 * - Distance-Angle Controller: Coordinates combined motion
 * 
 * The controller provides:
 * - Smooth transitions between motion types
 * - Coordinated linear and angular motion
 * - Configurable motion limits
 * - Independent control of each motion aspect
 */
class SimpleTripleBasicController: public BaseController {
    std::shared_ptr<BasicController> distanceController;
    std::shared_ptr<BasicController> distanceAngleController;
    std::shared_ptr<BasicController> angleController;
    std::shared_ptr<BaseRobot> robot;
    std::shared_ptr<TripleBasicParameters> params;
    std::function<double()> targetTranslational;
    std::function<double()> positionTranslational;
    std::function<Angle()> targetRotational;
    std::function<Angle()> positionRotational;
public:

    /**
     * @brief Constructs a new triple controller
     * 
     * @param robot Reference to robot instance
     * @param distanceController Linear motion controller
     * @param distanceAngleController Combined motion controller
     * @param angleController Angular motion controller
     * @param params Controller configuration parameters
     */
    SimpleTripleBasicController(
            const std::shared_ptr<BaseRobot> &robot,const std::shared_ptr<BasicController> &distanceController,
                                const std::shared_ptr<BasicController> &distanceAngleController,
                                const std::shared_ptr<BasicController> &angleController, const std::shared_ptr<TripleBasicParameters> &params,
                                const std::function<double()>& targetTranslational,
                                const std::function<double()>& positionTranslational,
                                const std::function<Angle()>& targetRotational,
                                const std::function<Angle()>& positionRotational);
    SimpleTripleBasicController(
            const std::shared_ptr<BaseRobot> &robot,const std::shared_ptr<BasicController> &distanceController,
                                const std::shared_ptr<BasicController> &distanceAngleController,
                                const std::shared_ptr<BasicController> &angleController, const std::shared_ptr<TripleBasicParameters> &params);

    /**
     * @brief Executes one control cycle
     * 
     * This method:
     * - Updates all three controllers
     * - Coordinates their outputs
     * - Applies motion commands to motors
     */
    void compute() override;

    /**
     * @brief Resets controller state
     * 
     * @param correct_error If true, maintains current error for smooth transition
     */
    void reset(bool correct_error) override;

    [[nodiscard]] std::shared_ptr<BasicController> getDistanceController() const;

    [[nodiscard]] std::shared_ptr<BasicController> getDistanceAngleController() const;

    [[nodiscard]] std::shared_ptr<BasicController> getAngleController() const;

    [[nodiscard]] std::shared_ptr<TripleBasicParameters> getParameters() const;

    void setDistanceController(const std::shared_ptr<BasicController> &distance_controller) {
        distanceController = distance_controller;
    }

    void setDistanceAngleController(const std::shared_ptr<BasicController> &distance_angle_controller) {
        distanceAngleController = distance_angle_controller;
    }

    void setAngleController(const std::shared_ptr<BasicController> &angle_controller) {
        angleController = angle_controller;
    }
};

/**
 * @brief JSON converter for TripleBasicParameters
 * 
 * Provides serialization and deserialization of controller
 * parameters to/from JSON format.
 */
ARDUINOJSON_BEGIN_PUBLIC_NAMESPACE
template <>
struct Converter<TripleBasicParameters> {
    /**
     * @brief Converts parameters to JSON
     * @param src Source parameters
     * @param dst Destination JSON object
     */
    static void toJson(const TripleBasicParameters& src, JsonVariant dst) {
        dst["speed_min_l"] = src.speed_min_l;
        dst["speed_min_r"] = src.speed_min_r;
        dst["maxValueDistance"] = src.maxValueDistance;
        dst["maxValueDistanceAngle"] = src.maxValueDistanceAngle;
        dst["maxValueAngular"] = src.maxValueAngular;
        dst["speed_mode"] = src.speed_mode;
    }

    /**
     * @brief Creates parameters from JSON
     * @param src Source JSON object
     * @return TripleBasicParameters Constructed parameters
     */
    static TripleBasicParameters fromJson(JsonVariantConst src) {
        if (src["speed_mode"].is<TripleController::SpeedMode>()) {
            return {src["speed_min_l"].as<double>(),src["speed_min_r"].as<double>(),src["maxValueDistance"].as<double>(),src["maxValueDistanceAngle"].as<double>(), src["maxValueAngular"].as<double>(), src["speed_mode"].as<TripleController::SpeedMode>()};
        }
        return {src["speed_min_l"].as<double>(),src["speed_min_r"].as<double>(),src["maxValueDistance"].as<double>(),src["maxValueDistanceAngle"].as<double>(), src["maxValueAngular"].as<double>()};
    }

    /**
     * @brief Validates JSON format
     * @param src JSON object to check
     * @return bool True if JSON is valid
     */
    static bool checkJson(JsonVariantConst src) {
        return src["speed_min_l"].is<double>() && src["speed_min_r"].is<double>() && src["maxValueDistance"].is<double>() && src["maxValueAngular"].is<double>() && src["maxValueDistanceAngle"].is<double>();
    }
};

ARDUINOJSON_END_PUBLIC_NAMESPACE

#endif //PAMITEENSY_SIMPLETRIPLEBASICCONTROLLER_H
