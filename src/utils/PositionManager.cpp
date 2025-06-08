//
// Created by fogoz on 24/04/2025.
//

#include "utils/PositionManager.h"
#include <tuple>

#include "robot/BaseRobot.h"
#include "utils/config.h"

PositionManager::PositionManager(const std::shared_ptr<BaseRobot> &robot,
                                 const std::shared_ptr<BaseEncoder> &leftWheelEncoder,
                                 const std::shared_ptr<BaseEncoder> &rightWheelEncoder,
                                 const std::shared_ptr<PositionParameters> &params, const std::shared_ptr<SpeedEstimator> &distanceEstimator, const std::shared_ptr<SpeedEstimator>&angleEstimator) : robot(robot), leftWheelEncoder(leftWheelEncoder),
                                                                     rightWheelEncoder(rightWheelEncoder),
                                                                     params(params), distanceEstimator(distanceEstimator), angleEstimator(angleEstimator){}

std::tuple<Position, double, double> PositionManager::computePosition(const Position& pos) {
    std::lock_guard lock(this->mutex);
    int32_t left_c = leftWheelEncoder->getDeltaCount();
    int32_t right_c = rightWheelEncoder->getDeltaCount();
    double left = left_c * params->left_wheel_diam;
    double right = right_c * params->right_wheel_diam;
    robot->update(left, right);

    double distance = (left + right)/2;
    double angle = (right-left)/params->track_mm;

    deltaDistance = distance;
    deltaAngle = angle;
    if (distanceEstimator != nullptr) {
        distanceEstimator->update(distance);
    }
    if (angleEstimator != nullptr) {
        angleEstimator->update(angle * RAD_TO_DEG);
    }
    if(angle == 0){
        deltaPos = pos.getSinCosAngle() * distance;
    }else{
        double r = distance*params->track_mm/(right-left);
        double angle_rad = pos.getAngle().toRadians();
        deltaPos = {r * (-sin(angle_rad) + sin(angle_rad + angle)), r * (cos(angle_rad) - cos(angle_rad + angle)), Angle::fromRadians(angle)};
    }
    return std::make_tuple((pos+deltaPos).warpAngle(), distance, angle);
}

Position PositionManager::getDeltaPos() const {
    std::lock_guard lock(this->mutex);
    return deltaPos;
}

double PositionManager::getDeltaDist() const {
    std::lock_guard lock(this->mutex);
    return deltaDistance;
}

Angle PositionManager::getDeltaAngle() const {
    std::lock_guard lock(this->mutex);
    return Angle::fromRadians(deltaAngle);
}

void PositionManager::overrideLeftRightEncoder(std::shared_ptr<BaseEncoder> leftEncoder,
    std::shared_ptr<BaseEncoder> rightEncoder, std::shared_ptr<PositionParameters> params) {
    std::lock_guard lock(this->mutex);
    this->leftWheelEncoder = leftEncoder;
    this->rightWheelEncoder = rightEncoder;
    this->params = params;
}
