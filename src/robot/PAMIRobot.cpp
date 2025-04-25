//
// Created by fogoz on 24/04/2025.
//

#include "robot/PAMIRobot.h"

double PAMIRobot::getDT() {
    return dt;
}

void PAMIRobot::computeTarget() {

}

void PAMIRobot::computePosition() {
    auto [pos, distance, angle] = positionManager->computePosition();
    this->pos = pos;
    this->translationPos += distance;
    this->rotationPos += angle;
}

void PAMIRobot::computeController() {
    controller->compute();
}

void PAMIRobot::addTarget(std::shared_ptr<BaseTarget> target) {
    targets.push_back(target);
}

void PAMIRobot::compute() {
    std::chrono::duration<double, std::ratio<1, 1>> _dt{};
    auto current = std::chrono::steady_clock::now();
    _dt = current - previous_time;
    previous_time= current;
    dt = _dt.count();
    computePosition();
    computeTarget();
    computeController();
}

void PAMIRobot::init(std::shared_ptr<PAMIRobot> robot) {
    previous_time = std::chrono::steady_clock::now();
}
