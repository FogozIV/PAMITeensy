//
// Created by fogoz on 03/06/2025.
//

#include "controller/calibration_methodo/ZieglerNicholsMethodoTriplePID.h"
#include <controller/SimpleTripleBasicController.h>
#include "basic_controller/PID.h"
#include "SD.h"
#include <utils/Mutex.h>
#include <utils/TaskScheduler.h>

void ZieglerNicholsMethodoTriplePID::start() {
    CalibrationMethodo::start();
    pid = std::make_shared<PID>(robot, this->initialValue, 0,0,1000);
    openFile();
    currentController = std::make_shared<SimpleTripleBasicController>(robot, pid, pid, pid, std::make_shared<TripleBasicParameters>());
    robot->setController(currentController);
    oscTracker = std::make_shared<OscillationTracker>();
    if(distance){
        robot->setTranslationalPosition(0);
        robot->setTranslationalTarget(400);
        robot->setDoneDistance(false);
        uint64_t value = 0;
        computeIndex = robot->addEndComputeHook([this](){
            oscTracker->update(robot->getTranslationalPosition(), robot->getDT() * 1000 * 1000);
            buffer->printf("%f; %f; %f\r\n", robot->getTranslationalTarget(), robot->getTranslationalPosition(), robot->getTranslationalTarget() - robot->getTranslationalPosition());
        });
    }else{
        robot->setRotationalPosition(AngleConstants::ZERO);
        robot->setRotationalTarget(AngleConstants::LEFT);
        robot->setDoneAngular(false);
        computeIndex = robot->addEndComputeHook([this](){
            oscTracker->update(robot->getRotationalPosition().toDegrees(), robot->getDT() * 1000 * 1000);
            buffer->printf("%f; %f; %f\r\n", robot->getRotationalTarget().toDegrees(), robot->getRotationalPosition().toDegrees(), (robot->getRotationalTarget() - robot->getRotationalPosition()).toDegrees());
        });
    }
    index = scheduler->addTask(seconds(10), [this](){
        streamSplitter.printf("Test kp : %f\r\n", pid->getKp());
        robot->setControlDisabled(true);
        robot->getEventEndOfComputeNotifier()->wait(); //dangerous because it can hang a thread that's why we use a threadpool
        streamSplitter.printf("Oscillating : %i\r\n", oscTracker->is_oscillating());
        if (oscTracker->is_oscillating()) {
            streamSplitter.printf("Amplitude %f\r\n", oscTracker->get_mean_amplitude());
            switch (oscTracker->get_amplitude_trend()) {
                case OscillationTracker::AmplitudeTrend::Increasing:
                    scheduler->addTask(milliseconds(1), [this]() {
                        this->stop();
                    });
                    streamSplitter.println(oscTracker->get_oscillation_period_s());
                    streamSplitter.println(pid->getKp());
                    streamSplitter.println("Amplitude Increasing");
                    break;
                case OscillationTracker::AmplitudeTrend::Decreasing:
                    streamSplitter.println("Amplitude Decreasing");
                    break;
                case OscillationTracker::AmplitudeTrend::Stable:
                    scheduler->addTask(milliseconds(1), [this]() {
                        this->stop();
                    });
                    streamSplitter.println(oscTracker->get_oscillation_period_s());
                    streamSplitter.println("Amplitude Stable");
                default:
                    streamSplitter.println("Unknown amplitude");

            }
        }
        oscTracker = std::make_shared<OscillationTracker>();
        pid->getKpRef() *= 1.2;
        openFile();
        if (distance) {

        }else {
            robot->setRotationalPosition(AngleConstants::ZERO);
            robot->setRotationalTarget(AngleConstants::LEFT);
        }
        robot->setControlDisabled(false);
    }, seconds(10));
    robot->setControlDisabled(false);
}

void ZieglerNicholsMethodoTriplePID::save() {

}

ZieglerNicholsMethodoTriplePID::ZieglerNicholsMethodoTriplePID(std::shared_ptr<BaseRobot> robot, std::shared_ptr<Mutex> mutex, bool distance) : CalibrationMethodo(robot, mutex), distance(distance) {

}

void ZieglerNicholsMethodoTriplePID::stop() {
    CalibrationMethodo::stop();
    scheduler->deleteTaskId(index);
    robot->removeEndComputeHook(computeIndex);
    robot->getEventEndOfComputeNotifier()->wait();
    robot->getLeftMotor()->setPWM(0);
    robot->getRightMotor()->setPWM(0);
    robot->setController(previous_controller);
}

void ZieglerNicholsMethodoTriplePID::setInitialValue(double value) {
    this->initialValue = value;
}

void ZieglerNicholsMethodoTriplePID::openFile() {
    printerCleanup();
    lock_guard lg(sdMutex);
    SD.begin(BUILTIN_SDCARD);
    String data = String("ZieglerNicholsKP=") + String(pid->getKp()) + String(".txt");
    f = SD.open(data.c_str(), FILE_WRITE_BEGIN);
    buffer = std::make_shared<BufferFilePrint>(f, sdMutex, 2048);
    bufferPrinters.add(buffer);
}

void ZieglerNicholsMethodoTriplePID::printStatus(Stream &stream) {
    stream.printf("Current kp %f \r\n", pid->getKp());

}
