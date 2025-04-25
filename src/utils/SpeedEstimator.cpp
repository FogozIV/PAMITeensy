//
// Created by fogoz on 24/04/2025.
//

#include "utils/SpeedEstimator.h"
#include "robot/BaseRobot.h"

SpeedEstimator::SpeedEstimator(double bandwidth) {
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

