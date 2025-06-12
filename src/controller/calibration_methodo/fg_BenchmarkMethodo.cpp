//
// Created by fogoz on 04/06/2025.
//

#include <controller/calibration_methodo/BenchmarkMethodo.h>
#include <basic_controller/PIDSpeedFeedForward.h>

#include "ramp/CalculatedQuadramp.h"
#include "target/AngleTarget.h"
#include "target/DistanceTarget.h"
#include "target/FunctionTarget.h"
#include "target/PositionTarget.h"
#include "target/RotateTowardTarget.h"


BenchmarkMethodo::BenchmarkMethodo(const std::shared_ptr<BaseRobot> &robot, const std::shared_ptr<Mutex> &sdMutex,
                                   BenchmarkMode benchmark_type): CalibrationMethodo(robot, sdMutex),
                                                                  benchmark_type(benchmark_type) {
    assert(robot->getRobotType() == PAMIRobotType);
    this->robot = std::static_pointer_cast<PAMIRobot>(robot);
}

void BenchmarkMethodo::save() {
}

void PROGMEM BenchmarkMethodo::printStatus(Stream &stream) {
    stream.printf("Benchmark In progress\r\n");
    stream.printf("Benchmark Type: %u\r\n", benchmark_type);
    stream.printf("Benchmark score : %f\r\n", error);
}

void PROGMEM BenchmarkMethodo::start() {
    CalibrationMethodo::start();
    streamSplitter.println("Starting benchmark of current controller");
    robot->clearTarget();
    robot->controllerClear();
    robot->resetTargetsCurvilinearAndAngular();
    this->openFile((String("BenchmarkController") + String(rtc_get()) + (benchmark_type == 0 ? "ANGLE" : (benchmark_type == 1 ? "DISTANCE" : "ANGLE_DISTANCE")) + ".bin").c_str(), 8192*2*2);
    switch (benchmark_type) {
        case ANGLE:
            streamSplitter.println("Benchmark in angle");
            buffer->write_raw((uint64_t )BinaryFileType::BENCHMARK_ANGLE_V_0_1);
            //buffer->printf("Current error; Total Error; Total DT; Current DT\r\n");
            benchmarkComputeHook = robot->addEndComputeHooks([this]() {
                if (!robot->isControlDisabled() && !robot->isDoneAngular()) {
                    double current_error = multAngle * pow((robot->getRotationalPosition() - robot->getRotationalTarget()).toDegrees(), 2) * robot->getDT();
                    error += current_error;
                    dt += robot->getDT();
                    lock_guard lg(bufferMutex);
                    buffer->write_raw(current_error);
                    buffer->write_raw(error);
                    buffer->write_raw(dt);
                    buffer->write_raw(robot->getDT());
                    buffer->write_raw(robot->getRotationalTarget().toDegrees());
                    buffer->write_raw(robot->getRotationalPosition().toDegrees());
                    buffer->write_raw(robot->getRotationalRampSpeed().toDegrees());
                    buffer->write_raw(robot->getRotationalEstimatedSpeed().toDegrees());
                    buffer->write_raw(robot->getRotationalOtherEstimatedSpeed().toDegrees());
                    //buffer->printf("%f; %f; %f; %f\r\n", current_error, error, dt, robot->getDT());
                }
            });
            break;
        case DISTANCE:
            streamSplitter.println("Benchmark in distance");
            //buffer->printf("Current error; Total Error; Total DT; Current DT\r\n");
            buffer->write_raw((uint64_t)BinaryFileType::BENCHMARK_DISTANCE_V_0_1);
            buffer->write_raw(static_cast<uint8_t>(robot->getControllerDistance()->getType()));
            benchmarkComputeHook = robot->addEndComputeHooks([this]() {
                if (!robot->isControlDisabled() && !robot->isDoneDistance()) {
                    double current_error=  multDistance * pow((robot->getTranslationalPosition() - robot->getTranslationalTarget()), 2) * robot->getDT();
                    error += current_error;
                    dt += robot->getDT();
                    lock_guard lg(bufferMutex);
                    buffer->write_raw(current_error);
                    buffer->write_raw(error);
                    buffer->write_raw(dt);
                    buffer->write_raw(robot->getDT());
                    buffer->write_raw(robot->getTranslationalPosition());
                    buffer->write_raw(robot->getTranslationalTarget());
                    buffer->write_raw(robot->getLeftMotor()->getPWM());
                    buffer->write_raw(robot->getRightMotor()->getPWM());
                    if(robot->getControllerDistance()->getType() == BasicControllerType::PID){
                        std::shared_ptr<PID> pid = std::static_pointer_cast<PID>(robot->getControllerDistance());
                        buffer->write_raw(pid->getUp());
                        buffer->write_raw(pid->getUi());
                        buffer->write_raw(pid->getUd());
                    }else if(robot->getControllerDistance()->getType() == BasicControllerType::PIDSpeedFeedForward){
                        std::shared_ptr<PIDSpeedFeedForward> pid = std::static_pointer_cast<PIDSpeedFeedForward>(robot->getControllerDistance());
                        buffer->write_raw(pid->getUp());
                        buffer->write_raw(pid->getUi());
                        buffer->write_raw(pid->getUd());
                        buffer->write_raw(pid->getUff());
                    }
                    //buffer->printf("%f; %f; %f; %f\r\n", current_error, error, dt, robot->getDT());
                }
            });
            break;
        case ANGLE_DISTANCE:
            buffer->write_raw((uint64_t )BinaryFileType::BENCHMARK_LEGACY_DISTANCE_ANGLE);
            streamSplitter.println("Benchmark in angle & distance");
            //buffer->printf("Current error; Total Error; Total DT; Current DT; Current Error Angle; Current Error Distance\r\n");
            benchmarkComputeHook = robot->addEndComputeHooks([this]() {
                if (!robot->isControlDisabled() && !robot->isDoneDistance() && !robot->isDoneAngular()) {
                    double current_error_distance = multDistance * pow((robot->getTranslationalPosition() - robot->getTranslationalTarget()), 2) * robot->getDT();
                    double current_error_angle = multAngle * pow((robot->getRotationalPosition() - robot->getRotationalTarget()).toDegrees(), 2) * robot->getDT();
                    error += current_error_distance + current_error_angle;
                    dt += robot->getDT();
                    lock_guard lg(bufferMutex);
                    buffer->write_raw(current_error_distance + current_error_angle);
                    buffer->write_raw(error);
                    buffer->write_raw(dt);
                    buffer->write_raw(robot->getDT());
                    buffer->write_raw(current_error_angle);
                    buffer->write_raw(current_error_distance);
                    buffer->write_raw(robot->getTranslationalPosition());
                    buffer->write_raw(robot->getTranslationalTarget());
                    buffer->write_raw(robot->getRotationalPosition().toDegrees());
                    buffer->write_raw(robot->getRotationalTarget().toDegrees());
                    buffer->write_raw(robot->getCurrentPosition().getX());
                    buffer->write_raw(robot->getCurrentPosition().getY());
                    //buffer->printf("%f; %f; %f; %f; %f; %f\r\n", current_error_distance + current_error_angle, error, dt, robot->getDT(), current_error_angle, current_error_distance);
                }
            });
            break;
    }
    allTargetHook = robot->addAllTargetEndedHooks([this]() {
        scheduler->addTask(microseconds(1), [this]() {
            this->stop();
        });
    });
    switch (benchmark_type) {
        case ANGLE:
            COMPLETE_ANGLE_TARGET(AngleConstants::LEFT, RampData(90,180));
            COMPLETE_ANGLE_TARGET(AngleConstants::RIGHT, RampData(90,180));
            COMPLETE_ANGLE_TARGET(AngleConstants::ZERO, RampData(180,180));
            robot->setDoneAngular(false);
            break;
        case DISTANCE:
            COMPLETE_DISTANCE_TARGET(1000, RampData(200,400));
            COMPLETE_DISTANCE_TARGET(-2000, RampData(200,400));
            COMPLETE_DISTANCE_TARGET(1000, RampData(200,400));
            robot->setDoneDistance(false);
            break;
        case ANGLE_DISTANCE:
            robot->reset_to({});//reset to 0,0
            COMPLETE_POSITION_TARGET((Position(100, 0)), RampData(100,200));
            COMPLETE_ROTATE_TOWARD_TARGET(Position(100,100), RampData(90,180));
            COMPLETE_POSITION_TARGET(Position(100,100), RampData(100,200));
            COMPLETE_ROTATE_TOWARD_TARGET(Position(0,100), RampData(90,180));
            COMPLETE_POSITION_TARGET(Position(0,100), RampData(100,200));
            COMPLETE_ROTATE_TOWARD_TARGET(Position(0,0), RampData(90,180));
            COMPLETE_POSITION_TARGET(Position(0,0), RampData(100,200));
            COMPLETE_ANGLE_TARGET(AngleConstants::ZERO, RampData(90,180));
            robot->setDoneAngular(false);
            robot->setDoneDistance(false);
            break;
    }
    robot->setControlDisabled(false);
}

void PROGMEM BenchmarkMethodo::stop() {
    streamSplitter.println("Stopping Benchmark");
    CalibrationMethodo::stop();
    streamSplitter.println("Done CalibrationMethodo::stop");
    robot->removeEndComputeHooks(benchmarkComputeHook);
    robot->removeAllTargetEndedHooks(allTargetHook);
    if (error == 0.0 && dt == 0.0) {
        streamSplitter.println("No benchmark data was collected.");
    }else {
        streamSplitter.printf("The error is : %f\r\nThe total time is %f\r\nThe error divided by the total time is %f\r\n", error, dt, error/dt);
    }

}

void BenchmarkMethodo::setMultDistance(double multDistance) {
    this->multDistance = multDistance;
}

void BenchmarkMethodo::setMultAngle(double multAngle) {
    this->multAngle = multAngle;
}
