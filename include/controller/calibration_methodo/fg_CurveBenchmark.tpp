//
// Created by fogoz on 12/06/2025.
//
#pragma once
#include <controller/calibration_methodo/CurveBenchmark.h>

#include "basic_controller/PIDFilteredD.h"

template<typename Ramp>
void CurveBenchmark<Ramp>::save() {

}

template<typename Ramp>
void CurveBenchmark<Ramp>::printStatus(Stream &stream) {
    stream.printf("Benchmark In progress\r\n");
    stream.printf("Benchmark score : %f\r\n", error);
}

template<typename Ramp>
CurveBenchmark<Ramp>::CurveBenchmark(const std::shared_ptr<BaseRobot> &robot, const std::shared_ptr<Mutex> &sdMutex, std::shared_ptr<ContinuousCurveTarget<Ramp>> curveTarget) : CalibrationMethodo(robot, sdMutex) {
    assert(robot->getRobotType() == PAMIRobotType);
    this->robot = std::static_pointer_cast<PAMIRobot>(robot);
    this->curveTarget = curveTarget;
}

void writeToBuffer(std::shared_ptr<BufferFilePrint> buffer, std::shared_ptr<BasicController> controller, std::shared_ptr<PAMIRobot> robot) {
    switch (controller->getType()) {
        case BasicControllerType::PID: {
            std::shared_ptr<PID> pid = std::static_pointer_cast<PID>(controller);
            buffer->write_raw(pid->getUp());
            buffer->write_raw(pid->getUi());
            buffer->write_raw(pid->getUd());
        }
            break;
        case BasicControllerType::PIDSpeedFeedForward: {
            std::shared_ptr<PIDSpeedFeedForward> pid = std::static_pointer_cast<PIDSpeedFeedForward>(controller);
            buffer->write_raw(pid->getUp());
            buffer->write_raw(pid->getUi());
            buffer->write_raw(pid->getUd());
            buffer->write_raw(pid->getUff());
        }
            break;
        case BasicControllerType::PIDFilteredD: {
            std::shared_ptr<PIDFilteredD> pid = std::static_pointer_cast<PIDFilteredD>(controller);
            buffer->write_raw(pid->getUp());
            buffer->write_raw(pid->getUi());
            buffer->write_raw(pid->getUd());
            buffer->write_raw(pid->getRawUd());
        }
            break;
        default:
            break;
    }
}

template<typename Ramp>
void CurveBenchmark<Ramp>::start() {
    CalibrationMethodo::start();
    streamSplitter.println("Starting benchmark of current controller");
    robot->clearTarget();
    robot->controllerClear();
    robot->resetTargetsCurvilinearAndAngular();
    this->openFile((String("BenchmarkCurve") + String(rtc_get())+ ".bin").c_str(), 8192*2*2);
    buffer->write_raw((uint64_t)BinaryFileType::BENCHMARK_LEGACY_CURVE);
    buffer->write_raw(static_cast<uint8_t>(robot->getControllerDistance()->getType()));
    buffer->write_raw(static_cast<uint8_t>(robot->getControllerDistanceAngle()->getType()));
    benchmarkComputeHook = robot->addEndComputeHooks([this]() {
        if (!robot->isControlDisabled() && !robot->isDoneDistance()) {
            double current_error = (curveTarget->getTargetPosition() - robot->getCurrentPosition()).getDistance();
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
            Position pos = robot->getCurrentPosition();
            buffer->write_raw(pos.getX());
            buffer->write_raw(pos.getY());
            buffer->write_raw(pos.getAngle().toDegrees());
            pos = curveTarget->getTargetPosition();
            buffer->write_raw(pos.getX());
            buffer->write_raw(pos.getY());
            writeToBuffer(buffer, robot->getControllerDistance(), robot);
            writeToBuffer(buffer, robot->getControllerDistanceAngle(), robot);
            //buffer->printf("%f; %f; %f; %f\r\n", current_error, error, dt, robot->getDT());
        }
    });
    allTargetHook = robot->addAllTargetEndedHooks([this]() {
        scheduler->addTask(microseconds(1), [this]() {
            this->stop();
        });
    });
    robot->getEventEndOfComputeNotifier()->wait();
    robot->clearTarget();
    robot->controllerClear();
    robot->addTarget(curveTarget);
    robot->setControlDisabled(false);
}


template<typename Ramp>
void CurveBenchmark<Ramp>::stop() {
    CalibrationMethodo::stop();
    robot->removeEndComputeHooks(benchmarkComputeHook);
    robot->removeAllTargetEndedHooks(allTargetHook);
    if (error == 0.0 && dt == 0.0) {
        streamSplitter.println("No benchmark data was collected.");
    }else {
        streamSplitter.printf("The error is : %f\r\nThe total time is %f\r\nThe error divided by the total time is %f\r\n", error, dt, error/dt);
    }
}
