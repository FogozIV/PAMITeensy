//
// Created by fogoz on 24/04/2025.
//
#include "robot/BaseRobot.h"

Position BaseRobot::getCurrentPosition() {
    return pos;
}

std::shared_ptr<Motor> BaseRobot::getLeftMotor() {
    return motorInversed? rightMotor: leftMotor;
}

std::shared_ptr<Motor> BaseRobot::getRightMotor() {
    return motorInversed ? leftMotor: rightMotor;
}

bool BaseRobot::isMotorInversed() {
    return motorInversed;
}

std::shared_ptr<BaseController> BaseRobot::getController() {
    return controller;
}

double BaseRobot::getTranslationalEstimatedSpeed() {
    return getAngleEstimator()->getSpeed();
}

double BaseRobot::getRotationalEstimatedSpeed() {
    return getAngleEstimator()->getSpeed();
}

std::shared_ptr<SpeedEstimator> BaseRobot::getDistanceEstimator() {
    return distanceSpeedEstimator;
}

std::shared_ptr<SpeedEstimator> BaseRobot::getAngleEstimator() {
    return angleSpeedEstimator;
}

void BaseRobot::setTranslationalPosition(double pos) {
    translationPos = pos;
}

void BaseRobot::setTranslationalTarget(double pos) {
    translationTarget = pos;
}

void BaseRobot::setRotationalPosition(double pos) {
    rotationPos = pos;
}

void BaseRobot::setRotationalTarget(double pos) {
    rotationTarget = pos;
}

double BaseRobot::getRotationalTarget() {
    return rotationPos;
}

double BaseRobot::getTranslationalTarget() {
    return translationTarget;
}

double BaseRobot::getRotationalPosition() {
    return rotationPos;
}

double BaseRobot::getTranslationalPosition() {
    return translationPos;
}

bool BaseRobot::isDoneDistance() {
    return done_distance;
}

bool BaseRobot::isDoneAngular() {
    return done_angular;
}

void BaseRobot::setDoneAngular(bool done) {
    done_angular = done;
}

void BaseRobot::setDoneDistance(bool done) {
    done_distance = done;
}

void BaseRobot::setTranslationalRampSpeed(double speed) {
    translationalSpeedRamp = speed;
}

void BaseRobot::setRotationalRampSpeed(double speed) {
    rotationalSpeedRamp = speed;
}

double BaseRobot::getTranslationalRampSpeed() {
    return translationalSpeedRamp;
}

double BaseRobot::getRotationalRampSpeed() {
    return rotationalSpeedRamp;
}

bool BaseRobot::isAutoBackward() {
    return auto_backward;
}

void BaseRobot::setAutoBackward(bool enabled) {
    auto_backward = enabled;
}
