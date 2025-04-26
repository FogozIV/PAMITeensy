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

void BaseRobot::beginCalibrationEncoder() {
    left_encoder_count = leftEncoder->getEncoderCount();
    right_encoder_count = rightEncoder->getEncoderCount();
}

void BaseRobot::endCalibrationAngleTurnEncoder(double turns) {
    endCalibrationAngleRadEncoder(turns * 360 * DEG_TO_RAD);
}

void BaseRobot::endCalibrationAngleDegEncoder(double angle) {
    endCalibrationAngleRadEncoder(angle*DEG_TO_RAD);
}

void BaseRobot::endCalibrationAngleRadEncoder(double angle) {
    positionManagerParameters->track_mm *= computeCalibrationAngleRadEncoder(angle);
    save();
}

void BaseRobot::endCalibrationStraightEncoder(double distance) {
    auto [left, right] = computeCalibrationStraightEncoder(distance);
    positionManagerParameters->left_wheel_diam *= left;
    positionManagerParameters->right_wheel_diam *= right;
    save();
}

double BaseRobot::computeCalibrationAngleRadEncoder(double angle) {
    int32_t d_e_l = leftEncoder->getEncoderCount() - left_encoder_count;
    int32_t d_e_r = rightEncoder->getEncoderCount() - right_encoder_count;
    double left = d_e_l * positionManagerParameters->left_wheel_diam;
    double right = d_e_r * positionManagerParameters->right_wheel_diam;
    double estimatedAngle = (right - left)/positionManagerParameters->track_mm;
    return estimatedAngle/angle;
}

std::tuple<double, double> BaseRobot::computeCalibrationStraightEncoder(double distance) {
    int32_t d_e_l = leftEncoder->getEncoderCount() - left_encoder_count;
    int32_t d_e_r = rightEncoder->getEncoderCount() - right_encoder_count;
    double left = d_e_l * positionManagerParameters->left_wheel_diam;
    double right = d_e_r * positionManagerParameters->right_wheel_diam;

    double corr_left = 1;
    double corr_right = 1;
    if (left * distance < 0) {
        corr_left *= -1;
        left *= -1;
    }
    if (right * distance < 0) {
        corr_right *= -1;
        right *= -1;
    }
    double multiplier = left/right;
    corr_right *= multiplier;
    right*= multiplier;

    double estimated_distance = (right + left)/2;
    double distance_mult = distance/estimated_distance;
    return std::make_tuple(left*distance_mult, right*distance_mult);
}

void BaseRobot::calibrateMotors() {
    getLeftMotor()->setPWM(0);
    getRightMotor()->setPWM(0);
    delay(1000);
    angleSpeedEstimator->reset();
    distanceSpeedEstimator->reset();
    leftMotor->setPWM(leftMotor->getMaxPWM() * 0.1);
    delay(1000);
    leftMotor->setPWM(0);
    computePosition();

    if (distanceSpeedEstimator->getRealDistance() > 0) {
        if (angleSpeedEstimator->getRealDistance() > 0) {
            motorInversed = !motorInversed;
        }
    }else {
        leftMotor->setInversed(!leftMotor->isInversed());
        if (angleSpeedEstimator->getRealDistance() > 0) {
        }else {
            motorInversed = !motorInversed;
        }
    }

    angleSpeedEstimator->reset();
    distanceSpeedEstimator->reset();
    rightMotor->setPWM(rightMotor->getMaxPWM() * 0.1);
    delay(1000);
    rightMotor->setPWM(0);
    computePosition();
    if (distanceSpeedEstimator->getRealDistance() < 0) {
        rightMotor->setInversed(!rightMotor->isInversed());
    }

    pos = {0,0,0};
    angleSpeedEstimator->reset();
    distanceSpeedEstimator->reset();


    getLeftMotor()->setPWM(-getLeftMotor()->getMaxPWM() * 0.1);
    getRightMotor()->setPWM(getRightMotor()->getMaxPWM() * 0.1);
    delay(2000);
    getLeftMotor()->setPWM(0);
    getRightMotor()->setPWM(0);
    save();
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


std::shared_ptr<AX12Handler> BaseRobot::getAX12Handler() const {
    return ax12Handler;
}

