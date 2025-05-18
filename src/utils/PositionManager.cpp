//
// Created by fogoz on 24/04/2025.
//

#include "utils/PositionManager.h"
#include "robot/BaseRobot.h"

PositionManager::PositionManager(const std::shared_ptr<BaseRobot> &robot,
                                 const std::shared_ptr<BaseEncoder> &leftWheelEncoder,
                                 const std::shared_ptr<BaseEncoder> &rightWheelEncoder,
                                 const std::shared_ptr<PositionParameters> &params) : robot(robot), leftWheelEncoder(leftWheelEncoder),
                                                                     rightWheelEncoder(rightWheelEncoder),
                                                                     params(params) {}

std::tuple<Position, double, double> PositionManager::computePosition() {
    auto pos = robot->getCurrentPosition();

    double left = leftWheelEncoder->getDeltaCount() * params->left_wheel_diam;
    double right = rightWheelEncoder->getDeltaCount() * params->right_wheel_diam;

    double distance = (left + right)/2;
    double angle = (right-left)/params->track_mm;

    deltaDistance = distance;
    deltaAngle = angle;

    robot->getDistanceEstimator()->update(distance);
    robot->getAngleEstimator()->update(angle * RAD_TO_DEG);

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
    return deltaPos;
}

double PositionManager::getDeltaDist() const {
    return deltaDistance;
}

Angle PositionManager::getDeltaAngle() const {
    return Angle::fromRadians(deltaAngle);
}
