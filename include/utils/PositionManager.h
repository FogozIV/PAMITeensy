//
// Created by fogoz on 24/04/2025.
//

#ifndef PAMITEENSY_POSITIONMANAGER_H
#define PAMITEENSY_POSITIONMANAGER_H

#include <memory>
#include "encoders/BaseEncoder.h"
#include "utils/SpeedEstimator.h"
#include "utils/Position.h"

struct PositionParameters{
    double track_mm;
    double left_wheel_diam;
    double right_wheel_diam;
};

class BaseRobot;
class PositionManager {
    std::shared_ptr<BaseRobot> robot;
    std::shared_ptr<BaseEncoder> leftWheelEncoder;
    std::shared_ptr<BaseEncoder> rightWheelEncoder;
    PositionParameters params;
public:
    PositionManager(const std::shared_ptr<BaseRobot> &robot, const std::shared_ptr<BaseEncoder> &leftWheelEncoder,
                    const std::shared_ptr<BaseEncoder> &rightWheelEncoder, const PositionParameters &params);

    std::tuple<Position, double, double> computePosition();
};


#endif //PAMITEENSY_POSITIONMANAGER_H
