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

struct PositionParameters{
    double track_mm = 10;
    double left_wheel_diam = 1;
    double right_wheel_diam = 1;
    void multiply(double factor) {
        left_wheel_diam *= factor;
        right_wheel_diam *= factor;
    }
    PositionParameters(double track_mm=10, double left_wheel_diam = 1, double right_wheel_diam = 1) : track_mm(track_mm), left_wheel_diam(left_wheel_diam), right_wheel_diam(right_wheel_diam) {};
};

class BaseRobot;
class PositionManager {
    std::shared_ptr<BaseRobot> robot;
    std::shared_ptr<BaseEncoder> leftWheelEncoder;
    std::shared_ptr<BaseEncoder> rightWheelEncoder;
    std::shared_ptr<PositionParameters> params;
    Position deltaPos;
    double deltaDistance = 0.0;
    double deltaAngle = 0.0;
    mutable std::mutex mutex;
    std::shared_ptr<SpeedEstimator> distanceEstimator;
    std::shared_ptr<SpeedEstimator> angleEstimator;

public:
    PositionManager(const std::shared_ptr<BaseRobot> &robot, const std::shared_ptr<BaseEncoder> &leftWheelEncoder,
                    const std::shared_ptr<BaseEncoder> &rightWheelEncoder, const std::shared_ptr<PositionParameters> &params, const std::shared_ptr<SpeedEstimator> &distanceEstimator = nullptr, const std::shared_ptr<SpeedEstimator>&angleEstimator = nullptr);

    std::tuple<Position, double, double> computePosition(const Position &pos);

    Position getDeltaPos() const;

    double getDeltaDist() const;

    Angle getDeltaAngle() const;

    void overrideLeftRightEncoder(std::shared_ptr<BaseEncoder> leftEncoder, std::shared_ptr<BaseEncoder> rightEncoder, std::shared_ptr<PositionParameters> params);
};

ARDUINOJSON_BEGIN_PUBLIC_NAMESPACE
template <>
struct Converter<PositionParameters> {
    static void toJson(const PositionParameters& src, JsonVariant dst) {
        dst["track_mm"] = src.track_mm;
        dst["left_wheel_diam"] = src.left_wheel_diam;
        dst["right_wheel_diam"] = src.right_wheel_diam;
    }

    static PositionParameters fromJson(JsonVariantConst src) {
        return {src["track_mm"].as<double>(), src["left_wheel_diam"], src["right_wheel_diam"]};
    }

    static bool checkJson(JsonVariantConst src) {
        return src["track_mm"].is<double>() && src["left_wheel_diam"].is<double>() && src["right_wheel_diam"].is<double>();
    }
};


ARDUINOJSON_END_PUBLIC_NAMESPACE


#endif //PAMITEENSY_POSITIONMANAGER_H
