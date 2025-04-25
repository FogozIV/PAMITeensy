//
// Created by fogoz on 24/04/2025.
//

#include "robot/PAMIRobot.h"

#include <encoders/QuadEncoderImpl.h>

#include "basic_controller/PID.h"

double PAMIRobot::getDT() {
    return dt;
}

void PAMIRobot::computeTarget() {
    if(!targets.empty()){
        targets.front()->call_init();
        targets.front()->process();
        if(targets.front()->is_done()){
            targets.front()->call_done();
            targets.pop_front();
        }
    }
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
    leftEncoder = std::make_shared<QuadEncoderImpl>(0,1,1);
    rightEncoder = std::make_shared<QuadEncoderImpl>(2,3,2);
    bool sd_present = SD.begin(BUILTIN_SDCARD);
    bool filled = false;
    if(sd_present){
        File data_file = SD.open("PAMIRobot.json", FILE_READ);
        if (data_file) {
            JsonDocument document;
            deserializeJson(document, data_file);
            if (document["pid_distance"].is<JsonObject>()) {
                pidDistance = getPIDFromJson(robot, document["pid_distance"].as<JsonObject>());
            }else {
                pidDistance = std::make_shared<PID>(robot, 20, 0,0, 1000);
            }
            if (document["pid_angle"].is<JsonObject>()) {
                pidAngle = getPIDFromJson(robot, document["pid_angle"].as<JsonObject>());
            }else {
                pidAngle = std::make_shared<PID>(robot, 20, 0,0, 1000);
            }
            if (document["pid_distance_angle"].is<JsonObject>()) {
                pidDistanceAngle = getPIDFromJson(robot, document["pid_distance_angle"].as<JsonObject>());
            }else {
                pidDistanceAngle = std::make_shared<PID>(robot, 20, 0,0, 1000);
            }
            if (document["triple_parameters"].is<TripleBasicParameters>()) {
                pidParameters = std::make_shared<TripleBasicParameters>(document["triple_parameters"].as<TripleBasicParameters>());
            }else {
                pidParameters = std::make_shared<TripleBasicParameters>();
            }
            controller = std::make_shared<SimpleTripleBasicController>(robot, pidDistance, pidDistanceAngle, pidAngle, pidParameters);
            if (document["distance_estimator_bandwidth"].is<double>()) {
                distanceSpeedEstimator = std::make_shared<SpeedEstimator>(robot, document["distance_estimator_bandwidth"].as<double>());
            }else {
                distanceSpeedEstimator = std::make_shared<SpeedEstimator>(robot, 80);
            }
            if (document["angle_estimator_bandwidth"].is<double>()) {
                angleSpeedEstimator = std::make_shared<SpeedEstimator>(robot, document["angle_estimator_bandwidth"].as<double>());
            }else {
                angleSpeedEstimator = std::make_shared<SpeedEstimator>(robot, 80);
            }

            filled = true;
        }
    }

    std::shared_ptr<SpeedEstimator> distanceSpeedEstimator; //robot, bandwidth
    std::shared_ptr<SpeedEstimator> angleSpeedEstimator;

    std::shared_ptr<Motor> leftMotor; //DIRPWMMotor (pwmPin, dirPin, inversed, resolution)
    std::shared_ptr<Motor> rightMotor;

    std::shared_ptr<PositionManager> positionManager; //robot, leftwheelencoder, rightwheelencoder, position_params

}
