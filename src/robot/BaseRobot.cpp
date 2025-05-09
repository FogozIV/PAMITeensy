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

Angle BaseRobot::getRotationalEstimatedSpeed() {
    return Angle::fromDegrees(getAngleEstimator()->getSpeed());
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
//#define DEBUG_COMPUTE_CALIB_STRAIGHT
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
#ifdef DEBUG_COMPUTE_CALIB_STRAIGHT
    Serial.printf("left encoder value : %d %d\r\n", leftEncoder->getEncoderCount(), left_encoder_count);
    Serial.printf("right encoder value : %d %d\r\n", rightEncoder->getEncoderCount(), right_encoder_count);
    Serial.printf("multiplier : %f\r\n", multiplier);
    Serial.printf("corr left, right : %f, %f\r\n", corr_left, corr_right);
    Serial.printf("estimated distance : %f\r\n", estimated_distance);
    Serial.printf("Real distance : %f\r\n", distance);
#endif

    return std::make_tuple(corr_left*distance_mult, corr_right*distance_mult);
}

void BaseRobot::calibrateMotors() {
    control_disabled = true;
    #define MOTEUR_POWER_VALUE_CALIB 0.2
    leftMotor->setPWM(0);
    rightMotor->setPWM(0);
    delay(1000);
    angleSpeedEstimator->reset();
    distanceSpeedEstimator->reset();
    leftMotor->setPWM(leftMotor->getMaxPWM() * MOTEUR_POWER_VALUE_CALIB);
    delay(1000);
    leftMotor->setPWM(0);
    computePosition();

    if (distanceSpeedEstimator->getRealDistance() > 0) {
        if (angleSpeedEstimator->getRealDistance() > 0) {
            motorInversed = true;
        }
    }else {
        leftMotor->setInversed(!leftMotor->isInversed());
        if (angleSpeedEstimator->getRealDistance() > 0) {
        }else {
            motorInversed = true;
        }
    }

    angleSpeedEstimator->reset();
    distanceSpeedEstimator->reset();
    rightMotor->setPWM(rightMotor->getMaxPWM() * MOTEUR_POWER_VALUE_CALIB);
    delay(1000);
    rightMotor->setPWM(0);
    computePosition();
    if (distanceSpeedEstimator->getRealDistance() < 0) {
        rightMotor->setInversed(!rightMotor->isInversed());
    }

    pos = {0,0,0};
    angleSpeedEstimator->reset();
    distanceSpeedEstimator->reset();


    getLeftMotor()->setPWM(-getLeftMotor()->getMaxPWM() * MOTEUR_POWER_VALUE_CALIB);
    getRightMotor()->setPWM(getRightMotor()->getMaxPWM() * MOTEUR_POWER_VALUE_CALIB);
    delay(2000);
    getLeftMotor()->setPWM(0);
    getRightMotor()->setPWM(0);
    save();
    control_disabled = false;
    #undef MOTEUR_POWER_VALUE_CALIB
}

void BaseRobot::setTranslationalPosition(double pos) {
    translationPos = pos;
}

void BaseRobot::setTranslationalTarget(double pos) {
    translationTarget = pos;
}

void BaseRobot::setRotationalPosition(Angle pos) {
    rotationPos = pos;
}

void BaseRobot::setRotationalTarget(Angle pos) {
    rotationTarget = pos;
}

Angle BaseRobot::getRotationalTarget() {
    return rotationTarget;
}


double BaseRobot::getTranslationalTarget() {
    return translationTarget;
}

Angle BaseRobot::getRotationalPosition() {
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

void BaseRobot::setRotationalRampSpeed(Angle speed) {
    rotationalSpeedRamp = speed;
}

double BaseRobot::getTranslationalRampSpeed() {
    return translationalSpeedRamp;
}

Angle BaseRobot::getRotationalRampSpeed() {
    return rotationalSpeedRamp ;
}


std::shared_ptr<AX12Handler> BaseRobot::getAX12Handler() const {
    return ax12Handler;
}

void BaseRobot::setControlDisabled(bool value) {
    control_disabled = value;
}

bool BaseRobot::isControlDisabled() const {
    return control_disabled;
}

