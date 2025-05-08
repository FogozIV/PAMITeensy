//
// Created by fogoz on 24/04/2025.
//

#include <utility>

#include "utils/SpeedEstimator.h"
#include "robot/BaseRobot.h"

SpeedEstimator::SpeedEstimator(std::shared_ptr<BaseRobot> baseRobot, double bandwidth) : baseRobot(std::move(baseRobot)){
    this->bandwidth = ConditionalVariable(bandwidth, [this](double n) {
        this->setBandwidth(n);
    });
    setBandwidth(bandwidth);
    reset();
}

void SpeedEstimator::setBandwidth(double bandwidth) {
    this->kp = 2.0f * bandwidth;
    this->ki = 0.25f * this->kp * this->kp;
}

void SpeedEstimator::reset() {
    distance_estimation = 0.0f;
    speed = 0.0f;
    real_distance = 0.0f;
}

double SpeedEstimator::getSpeed() const {
    return speed;
}

double SpeedEstimator::getDistanceEstimation() const {
    return distance_estimation;
}

double SpeedEstimator::getRealDistance() const {
    return real_distance;
}

double SpeedEstimator::getBandwidth() const {
    return static_cast<double>(bandwidth);
}

ConditionalVariable & SpeedEstimator::getBandwidthRef() {
    return bandwidth;
}

void SpeedEstimator::update(double distance) {
    double deltaT = baseRobot->getDT();
    distance_estimation += deltaT * speed;

    real_distance += distance;
    double deltaPos = real_distance - distance_estimation;

    distance_estimation += deltaT * kp * deltaPos;
    speed += deltaT * ki * deltaPos;

    if (std::abs(speed) < 0.5f * deltaT * ki)
        speed = 0.0f;

}

