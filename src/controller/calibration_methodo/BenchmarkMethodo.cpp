//
// Created by fogoz on 04/06/2025.
//

#include <controller/calibration_methodo/BenchmarkMethodo.h>

#include "ramp/CalculatedQuadramp.h"
#include "target/AngleTarget.h"
#include "target/DistanceTarget.h"
#include "target/FunctionTarget.h"
#include "target/PositionTarget.h"
#include "target/RotateTowardTarget.h"

BenchmarkMethodo::BenchmarkMethodo(const std::shared_ptr<BaseRobot> &robot, const std::shared_ptr<Mutex> &sdMutex,
                                   BenchmarkMode benchmark_type): CalibrationMethodo(robot, sdMutex),
                                                                  benchmark_type(benchmark_type) {
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
    this->openFile("BenchmarkController.txt", 8192);
    switch (benchmark_type) {
        case ANGLE:
            streamSplitter.println("Benchmark in angle");
            buffer->printf("Current error; Total Error; Total DT; Current DT\r\n");
            benchmarkComputeHook = robot->addEndComputeHooks([this]() {
                if (!robot->isControlDisabled() && !robot->isDoneAngular()) {
                    double current_error = multAngle * pow((robot->getRotationalPosition() - robot->getRotationalTarget()).toDegrees(), 2) * robot->getDT();
                    error += current_error;
                    dt += robot->getDT();
                    lock_guard lg(bufferMutex);
                    buffer->printf("%f; %f; %f; %f\r\n", current_error, error, dt, robot->getDT());
                }
            });
            break;
        case DISTANCE:
            streamSplitter.println("Benchmark in distance");
            buffer->printf("Current error; Total Error; Total DT; Current DT\r\n");
            benchmarkComputeHook = robot->addEndComputeHooks([this]() {
                if (!robot->isControlDisabled() && !robot->isDoneDistance()) {
                    double current_error=  multDistance * pow((robot->getTranslationalPosition() - robot->getTranslationalTarget()), 2) * robot->getDT();
                    error += current_error;
                    dt += robot->getDT();
                    lock_guard lg(bufferMutex);
                    buffer->printf("%f; %f; %f; %f\r\n", current_error, error, dt, robot->getDT());
                }
            });
            break;
        case ANGLE_DISTANCE:
            streamSplitter.println("Benchmark in angle & distance");
            buffer->printf("Current error; Total Error; Total DT; Current DT; Current Error Angle; Current Error Distance\r\n");
            benchmarkComputeHook = robot->addEndComputeHooks([this]() {
                if (!robot->isControlDisabled() && !robot->isDoneDistance() && !robot->isDoneAngular()) {
                    double current_error_distance = multDistance * pow((robot->getTranslationalPosition() - robot->getTranslationalTarget()), 2) * robot->getDT();
                    double current_error_angle = multAngle * pow((robot->getRotationalPosition() - robot->getRotationalTarget()).toDegrees(), 2) * robot->getDT();
                    error += current_error_distance + current_error_angle;
                    dt += robot->getDT();
                    lock_guard lg(bufferMutex);
                    buffer->printf("%f; %f; %f; %f; %f; %f\r\n", current_error_distance + current_error_angle, error, dt, robot->getDT(), current_error_angle, current_error_distance);
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
            break;
        case DISTANCE:
            COMPLETE_DISTANCE_TARGET(100, RampData(100,200));
            COMPLETE_DISTANCE_TARGET(-200, RampData(100,200));
            COMPLETE_DISTANCE_TARGET(100, RampData(100,200));
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
