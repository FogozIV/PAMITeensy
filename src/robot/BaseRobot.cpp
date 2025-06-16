//
// Created by fogoz on 24/04/2025.
//
#include "robot/BaseRobot.h"

#include <chrono>
#include <utility>


#include "encoders/MotoEncoderParameterEstimation.h"
#include "utils/InteractContext.h"
#include "utils/RecursiveLeastSquare.h"

std::shared_ptr<PositionParameters> BaseRobot::getPositionManagerParameters() const {
    return positionManagerParameters;
}

RobotType BaseRobot::getRobotType() const {
    return robotType;
}

std::shared_ptr<RobotTolerance> BaseRobot::getTolerances() {
    return this->tolerances;
}


BaseRobot::BaseRobot(RobotType robotType, std::shared_ptr<Mutex> motorUpdate): robotType(robotType){
    this->motorUpdate = motorUpdate;
}

Position BaseRobot::getCurrentPosition() {
    std::lock_guard lock(this->positionMutex);
    return pos;
}

Position BaseRobot::getMotorPosition() {
    return motorPos;
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

void BaseRobot::setController(std::shared_ptr<BaseController> controller){
    this->controller = controller;
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

std::shared_ptr<SpeedEstimator> BaseRobot::getWheelDistanceEstimator() {
    return wheelDistanceSpeedEstimator;
}

std::shared_ptr<SpeedEstimator> BaseRobot::getWheelAngleEstimator() {
    return wheelAngleSpeedEstimator;
}

void BaseRobot::beginCalibrationEncoder() {
    left_encoder_count = leftEncoder->getEncoderCount();
    right_encoder_count = rightEncoder->getEncoderCount();
}

void BaseRobot::calibrateMotorEncoder(Stream& stream, std::shared_ptr<BaseRobot> robot) {
    robot->setControlDisabled(true);
    robot->distanceSpeedEstimator->reset();
    robot->angleSpeedEstimator->reset();
    RecursiveLeastSquares3x3 result;
    auto time_point = std::chrono::steady_clock::now();
    enterInteractContext(robot, stream, [&result, robot, &time_point, &stream] {
        static double previousA = 0;
        static double previousB = 0;
        static int32_t previous_left_encoder = 0;
        static int32_t previous_right_encoder = 0;
        if (time_point + std::chrono::milliseconds(1000) < std::chrono::steady_clock::now()) {
            robot->getPositionMutex().lock();
            DataPoint dp = {
                robot->distanceSpeedEstimator->getRealDistance() - previousA,
                (robot->angleSpeedEstimator->getRealDistance()-previousB)*DEG_TO_RAD,
                robot->getLeftWheelEncoderValue() - previous_left_encoder,
                robot->getRightWheelEncoderValue() - previous_right_encoder
            };
            result.add(dp);
            previousA = robot->distanceSpeedEstimator->getRealDistance();
            previousB = robot->angleSpeedEstimator->getRealDistance();
            previous_left_encoder = robot->getLeftWheelEncoderValue();
            previous_right_encoder = robot->getRightWheelEncoderValue();
            robot->getPositionMutex().unlock();
            time_point = std::chrono::steady_clock::now();
            auto params = result.getParams();
            if (params == nullptr) {
                stream.println("Matrix not ready");
                return;
            }
            auto left_diam = params->left_wheel_diam;
            auto right_diam = params->right_wheel_diam;
            auto track_mm = params->track_mm;
            stream.printf("Left wheel diameter : %f, Right wheel diam : %f, Track mm: %f\r\n", left_diam, right_diam, track_mm);
            stream.printf("Distance : %f, angle : %f, left : %d, right %d\r\n", dp.distance, dp.angle, dp.tic_left, dp.tic_right);
        }

    });
    auto params = result.getParams();
    if (params == nullptr) {
        stream.println("Calibration failed");
        return;
    }
    auto left_diam = params->left_wheel_diam;
    auto right_diam = params->right_wheel_diam;
    auto track_mm = params->track_mm;
    robot->wheelPositionManagerParameters->left_wheel_diam = left_diam;
    robot->wheelPositionManagerParameters->right_wheel_diam = right_diam;
    robot->wheelPositionManagerParameters->track_mm = track_mm;
    stream.printf("Saving Left wheel diameter : %f, Right wheel diam : %f, Track mm: %f\r\n", left_diam, right_diam, track_mm);
    robot->save();
}

void BaseRobot::setEncoderToMotors() {
    this->positionManager->overrideLeftRightEncoder(leftWheelEncoder, rightWheelEncoder, wheelPositionManagerParameters);
}

void BaseRobot::setEncoderToFreeWheel() {
    this->positionManager->overrideLeftRightEncoder(leftEncoder, rightEncoder, positionManagerParameters);
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

double FLASHMEM BaseRobot::computeCalibrationAngleRadEncoder(double angle) {
    int32_t d_e_l = leftEncoder->getEncoderCount() - left_encoder_count;
    int32_t d_e_r = rightEncoder->getEncoderCount() - right_encoder_count;
    return computeCalibrationAngleRadEncoder(d_e_l, d_e_r, angle);
}

double FLASHMEM BaseRobot::computeCalibrationAngleRadEncoder(int32_t d_e_l, int32_t d_e_r, double angle) {
    double left = d_e_l * positionManagerParameters->left_wheel_diam;
    double right = d_e_r * positionManagerParameters->right_wheel_diam;
    double estimatedAngle = (right - left)/positionManagerParameters->track_mm;
    return estimatedAngle/angle;
}

std::tuple<double, double> FLASHMEM BaseRobot::computeCalibrationStraightEncoder(double distance) {
    int32_t d_e_l = leftEncoder->getEncoderCount() - left_encoder_count;
    int32_t d_e_r = rightEncoder->getEncoderCount() - right_encoder_count;
//#define DEBUG_COMPUTE_CALIB_STRAIGHT
    return computeCalibrationStraightEncoder(d_e_l, d_e_r, distance);
}


std::tuple<double, double> FLASHMEM BaseRobot::computeCalibrationStraightEncoder(int32_t d_e_l, int32_t d_e_r, double distance) {
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


void FLASHMEM BaseRobot::calibrateMotors() {
    bool previous_control = control_disabled;
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

    this->positionMutex.lock();
    pos = {0,0,AngleConstants::ZERO};
    this->positionMutex.unlock();
    angleSpeedEstimator->reset();
    distanceSpeedEstimator->reset();


    getLeftMotor()->setPWM(-getLeftMotor()->getMaxPWM() * MOTEUR_POWER_VALUE_CALIB);
    getRightMotor()->setPWM(getRightMotor()->getMaxPWM() * MOTEUR_POWER_VALUE_CALIB);
    delay(2000);
    getLeftMotor()->setPWM(0);
    getRightMotor()->setPWM(0);
    save();
    #undef MOTEUR_POWER_VALUE_CALIB
    control_disabled = previous_control;
}

void BaseRobot::setTranslationalPosition(double pos) {
    translationPos = pos;
}

void BaseRobot::setTranslationalTarget(double pos) {
    translationTarget = pos;
}

void BaseRobot::setRotationalPosition(Angle pos) {
    rotationPos = std::move(pos);
}

void BaseRobot::setRotationalTarget(Angle pos) {
    rotationTarget = std::move(pos);
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

Mutex & BaseRobot::getPositionMutex() const {
    return positionMutex;
}

void BaseRobot::lockMotorMutex() const {
    if (this->motorUpdate != nullptr) {
        this->motorUpdate->lock();
    }
}

void BaseRobot::unlockMotorMutex() const {
    if (this->motorUpdate != nullptr) {
        this->motorUpdate->unlock();
    }
}

int32_t BaseRobot::getLeftEncoderValue() {
    return leftEncoder->getEncoderCount();
}

int32_t BaseRobot::getRightEncoderValue() {
    return rightEncoder->getEncoderCount();
}

int32_t BaseRobot::getRightWheelEncoderValue() {
    return rightWheelEncoder->getEncoderCount();
}

int32_t BaseRobot::getLeftWheelEncoderValue() {
    return leftWheelEncoder->getEncoderCount();
}

#define CALLBACK(name)\
    void BaseRobot::call##name(){\
        (name).call();\
    }\
    uint64_t BaseRobot::add##name(std::function<void()> fct){\
        return (name).addCallback(fct);\
    }\
    void BaseRobot::remove##name(uint64_t id){\
        (name).removeCallback(id);\
    }

CALLBACKS_LIST
std::shared_ptr<EventNotifierAndWaiter> BaseRobot::getEventEndOfComputeNotifier(){
    return endOfComputeNotifier;
}

void BaseRobot::resetTargetsCurvilinearAndAngular() {
    this->setTranslationalTarget(this->getTranslationalPosition());
    this->setRotationalTarget(this->getRotationalPosition());
    this->setRotationalRampSpeed(AngleConstants::ZERO);
    this->setTranslationalRampSpeed(0);
}

void BaseRobot::resetCalibrationEncoderList() {
    this->calibrations.clear();
}

void BaseRobot::addCalibrationData(double data) {
    this->calibrations.emplace_back(leftEncoder->getEncoderCount() - left_encoder_count, rightEncoder->getEncoderCount() - right_encoder_count, data);
}

void BaseRobot::finalizeCalibrationForwardLS() {
    RecursiveLeastSquare<2> rls;
    for (auto& data : this->calibrations) {
        int32_t left = std::get<0>(data);
        int32_t right = std::get<1>(data);
        double distance = std::get<2>(data);
        rls.addEquation({{static_cast<double>(left), static_cast<double>(right)}}, 2*distance);
    }
    auto result = rls.computeResult();
    if (!result.has_value()) {
        streamSplitter.println("Error while computing result");
        return;
    }
    auto rslt = result.value();
    positionManagerParameters->left_wheel_diam = rslt(0,0);
    positionManagerParameters->right_wheel_diam = rslt(1,0);
}

void BaseRobot::finalizeCalibrationRotationLS() {
    RecursiveLeastSquare<1> rls;
    for (auto& data : this->calibrations) {
        int32_t left = std::get<0>(data);
        int32_t right = std::get<1>(data);
        double angle = std::get<2>(data);
        angle *= 2 * M_PI;
        double left_d = left * positionManagerParameters->left_wheel_diam;
        double right_d = right * positionManagerParameters->right_wheel_diam;
        rls.addEquation({{angle}}, right_d - left_d);
    }
    auto result = rls.computeResult();
    if (!result.has_value()) {
        streamSplitter.println("Error while computing result");
        return;
    }
    auto rslt = result.value();
    positionManagerParameters->track_mm = rslt(0,0);
}

void BaseRobot::finalizeCalibrationForward() {
    int32_t cumul_left = 0;
    int32_t cumul_right = 0;
    double cumul_distance = 0;
    for(auto& data : this->calibrations){
        int32_t left = std::get<0>(data);
        int32_t right = std::get<1>(data);
        double distance = std::get<2>(data);
        if(distance < 0){
            left *= -1;
            right *= -1;
            distance *= -1;
        }
        cumul_left += left;
        cumul_right += right;
        cumul_distance += distance;
    }
    streamSplitter.printf("The encoder ticks are the following : %d, %d with a total distance of %f\r\n", cumul_left, cumul_right, cumul_distance);
    auto [left, right] = computeCalibrationStraightEncoder(cumul_left, cumul_right, cumul_distance);
    positionManagerParameters->left_wheel_diam *= left;
    positionManagerParameters->right_wheel_diam *= right;
}

void BaseRobot::finalizeCalibrationRotation() {
    int32_t cumul_left = 0;
    int32_t cumul_right = 0;
    double cumul_distance = 0;
    for(auto& data : this->calibrations){
        int32_t left = std::get<0>(data);
        int32_t right = std::get<1>(data);
        double distance = std::get<2>(data);
        if(distance < 0){
            left *= -1;
            right *= -1;
            distance *= -1;
        }
        cumul_left += left;
        cumul_right += right;
        cumul_distance += distance;
    }
    streamSplitter.printf("The encoder ticks are the following : %u, %u with a total distance of %f\r\n", cumul_left, cumul_right, cumul_distance);
    auto track = computeCalibrationAngleRadEncoder(cumul_left, cumul_right, cumul_distance*2*M_PI);
    this->positionManagerParameters->track_mm *= track;
}

void BaseRobot::printCalibrationParameters(Stream &stream) {
    stream.printf("The calibration parameters are the following \r\n "
                  "left_wheel (mm/tick) %f\r\n"
                  "right_wheel (mm/tick) %f\r\n"
                  "track (mm) %f\r\n", positionManagerParameters->left_wheel_diam, positionManagerParameters->right_wheel_diam, positionManagerParameters->track_mm);

}
