//
// Created by fogoz on 03/06/2025.
//

#include "controller/calibration_methodo/ZieglerNicholsMethodoTriplePID.h"
#include <controller/SimpleTripleBasicController.h>
#include "basic_controller/PID.h"
#include "SD.h"
#include <utils/Mutex.h>
#include <utils/TaskScheduler.h>

void printGainResult(double ku, double tu){
    streamSplitter.printf("Ziegler nichols controllers:\r\n"
                          "P type : P= %f\r\n"
                          "PI type : P= %f I= %f\r\n"
                          "PD type : P= %f D= %f\r\n"
                          "classic PID type : P= %f I= %f D= %f\r\n"
                          "Pessen Integral Rule : P= %f I= %f D= %f\r\n"
                          "some overshoot : P= %f I= %f D= %f\r\n"
                          "no overshoot : P= %f I= %f D= %f\r\n",
                          0.5*ku, 0.45*ku, 0.54*ku/tu, 0.8*ku, 0.1*ku*tu, 0.6*ku, 1.2*ku/tu, 0.075*ku*tu, 07 *ku, 1.75*ku/tu, 0.105*ku*tu, 0.33*ku, 0.66*ku/tu, 0.11*ku*tu, 0.2*ku, 0.4*ku/tu, 0.066*ku*tu);
}

// P= 100.200000 I= 785.956327 D= 8.431036
void PROGMEM ZieglerNicholsMethodoTriplePID::start() {
    CalibrationMethodo::start();
    pid = std::make_shared<PID>(robot, this->initialValue, 0,0,1000);
    openFile();
    currentController = std::make_shared<SimpleTripleBasicController>(robot, pid, pid, pid, std::make_shared<TripleBasicParameters>());
    robot->setController(currentController);
    oscTracker = std::make_shared<OscillationTracker>();
    if(distance){
        robot->setTranslationalPosition(0);
        robot->setTranslationalTarget(target);
        robot->setDoneDistance(false);
        computeIndex = robot->addEndComputeHooks([this](){
            oscTracker->update(robot->getTranslationalPosition(), robot->getDT() * 1000 * 1000);
            buffer->printf("%f; %f; %f\r\n", robot->getTranslationalTarget(), robot->getTranslationalPosition(), robot->getTranslationalTarget() - robot->getTranslationalPosition());
        });
    }else{
        robot->setRotationalPosition(AngleConstants::ZERO);
        robot->setRotationalTarget(Angle::fromDegrees(target));
        robot->setDoneAngular(false);
        computeIndex = robot->addEndComputeHooks([this](){
            oscTracker->update(robot->getRotationalPosition().toDegrees(), robot->getDT() * 1000 * 1000);
            buffer->printf("%f; %f; %f\r\n", robot->getRotationalTarget().toDegrees(), robot->getRotationalPosition().toDegrees(), (robot->getRotationalTarget() - robot->getRotationalPosition()).toDegrees());
        });
    }
    forward = true;
    index = scheduler->addTask(seconds(10), [this](){
        robot->setControlDisabled(true);
        streamSplitter.println("Waiting for end of compute notifier");
        robot->getEventEndOfComputeNotifier()->wait(); //dangerous because it can hang a thread that's why we use a threadpool
        streamSplitter.println("Ended compute");
        callbackManager.call();
        streamSplitter.printf("Oscillating : %i\r\n", oscTracker->is_oscillating());
        if (oscTracker->is_oscillating()) {
            streamSplitter.printf("Amplitude %f\r\n", oscTracker->get_mean_amplitude());
            switch (oscTracker->get_amplitude_trend()) {
                case OscillationTracker::AmplitudeTrend::Increasing:
                    streamSplitter.printf("The ultimate gain seems to be around %f, the period is about %f\r\n", pid->getKp(), oscTracker->get_oscillation_period_s());
                    streamSplitter.println("Amplitude Increasing");
                    printGainResult(this->pid->getKp(), oscTracker->get_oscillation_period_s());
                    scheduler->addTask(microseconds(1), [this]() {
                        this->stop();
                    });
                    return;
                case OscillationTracker::AmplitudeTrend::Decreasing:
                    streamSplitter.println("Amplitude Decreasing");
                    break;
                case OscillationTracker::AmplitudeTrend::Stable:
                    streamSplitter.printf("The ultimate gain seems to be around %f, the period is about %f\r\n", pid->getKp(), oscTracker->get_oscillation_period_s());
                    streamSplitter.println("Amplitude Stable");
                    printGainResult(this->pid->getKp(), oscTracker->get_oscillation_period_s());
                    scheduler->addTask(microseconds(1), [this]() {
                        this->stop();
                    });
                    return;
                default:
                    streamSplitter.println("Unknown amplitude");

            }
        }
        oscTracker = std::make_shared<OscillationTracker>();
        pid->getKpRef() *= multiplier;
        streamSplitter.printf("Testing new kp : %f\r\n", pid->getKp());
        openFile();
        if (distance) {
            if (forward) {
                robot->setTranslationalTarget(0);
            }else {
                robot->setTranslationalTarget(target);
            }
            forward = !forward;
        }else {
            if (forward) {
                robot->setRotationalTarget(AngleConstants::ZERO);
            }else {
                robot->setRotationalTarget(Angle::fromDegrees(target));
            }
            forward = !forward;
        }
        robot->setControlDisabled(false);
    }, seconds(10));
    robot->setControlDisabled(false);
}

void ZieglerNicholsMethodoTriplePID::save() {

}

ZieglerNicholsMethodoTriplePID::ZieglerNicholsMethodoTriplePID(std::shared_ptr<BaseRobot> robot, std::shared_ptr<Mutex> mutex, bool distance) : CalibrationMethodo(robot, mutex), distance(distance) {
    if (distance) {
        target = 200;
    }else {
        target = 45;
    }
}

void PROGMEM ZieglerNicholsMethodoTriplePID::stop() {
    CalibrationMethodo::stop();
    scheduler->deleteTaskId(index);
    robot->removeEndComputeHooks(computeIndex);
    callbackManager.clear();
}

void ZieglerNicholsMethodoTriplePID::setInitialValue(double value) {
    this->initialValue = value;
}

void ZieglerNicholsMethodoTriplePID::openFile() {
    String data = String("ZieglerNicholsKP=") + String(pid->getKp()) + String("step=") + String(target);
    if (distance) {
        data += "distance";
    }
    data += ".txt";
    CalibrationMethodo::openFile(data.c_str());
}

void ZieglerNicholsMethodoTriplePID::printStatus(Stream &stream) {
    stream.printf("Current kp %f \r\n", pid->getKp());
}

void ZieglerNicholsMethodoTriplePID::setTarget(double value) {
    this->target = value;
}

void ZieglerNicholsMethodoTriplePID::setMultiplier(double multiplier) {
    this->multiplier = multiplier;
}
