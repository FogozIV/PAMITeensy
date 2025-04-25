//
// Created by fogoz on 24/04/2025.
//

#ifndef PAMITEENSY_POSITIONMANAGER_H
#define PAMITEENSY_POSITIONMANAGER_H

#include <memory>
#include "ArduinoJson.h"

#include "encoders/BaseEncoder.h"
#include "utils/SpeedEstimator.h"
#include "utils/Position.h"

struct PositionParameters{
    double track_mm = 10;
    double left_wheel_diam = 1;
    double right_wheel_diam = 1;
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

public:
    PositionManager(const std::shared_ptr<BaseRobot> &robot, const std::shared_ptr<BaseEncoder> &leftWheelEncoder,
                    const std::shared_ptr<BaseEncoder> &rightWheelEncoder, const std::shared_ptr<PositionParameters> &params);

    std::tuple<Position, double, double> computePosition();

    Position getDeltaPos() const;

    double getDeltaDist() const;

    double getDeltaAngle() const;
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
