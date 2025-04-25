//
// Created by fogoz on 24/04/2025.
//

#include "utils/PositionManager.h"
#include "robot/BaseRobot.h"

PositionManager::PositionManager(const std::shared_ptr<BaseRobot> &robot,
                                 const std::shared_ptr<BaseEncoder> &leftWheelEncoder,
                                 const std::shared_ptr<BaseEncoder> &rightWheelEncoder,
                                 const PositionParameters &params) : robot(robot), leftWheelEncoder(leftWheelEncoder),
                                                                     rightWheelEncoder(rightWheelEncoder),
                                                                     params(params) {}

std::tuple<Position, double, double> PositionManager::computePosition() {
    auto pos = robot->getCurrentPosition();

    double left = leftWheelEncoder->getDeltaCount() * params.left_wheel_diam;
    double right = rightWheelEncoder->getDeltaCount() * params.right_wheel_diam;

    double distance = (left + right)/2;
    double angle = (right-left)/params.track_mm;

    robot->getDistanceEstimator()->update(distance);
    robot->getAngleEstimator()->update(angle);

    if(angle == 0){
        pos += {cos(pos.getAngleRad()) * distance, sin(pos.getAngleRad()) * distance};
    }else{
        double r = distance*params.track_mm/(right-left);
        pos+= {r * (-sin(pos.getAngleRad()) + sin(pos.getAngleRad() + angle)), r * (cos(pos.getAngleRad()) - cos(pos.getAngleRad() + angle)), angle};
    }
    return std::make_tuple(pos, distance, angle);
}
