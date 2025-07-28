//
// Created by fogoz on 03/06/2025.
//

#include "controller/calibration_methodo/ZieglerNicholsMethodoTriplePID.h"
#include <controller/SimpleTripleBasicController.h>
#include "basic_controller/PID.h"
#include "SD.h"
#include <utils/Mutex.h>
#include <utils/TaskScheduler.h>

#include "basic_controller/BasicControllerFactory.h"
#include "robot/PAMIRobot.h"

void PROGMEM printGainResult(double ku, double tu){
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
    auto params = std::make_shared<TripleBasicParameters>();
    params->speed_mode = speed;
    currentController = std::make_shared<SimpleTripleBasicController>(robot, pid, pid, pid, params);

    robot->setController(currentController);
    oscTracker = std::make_shared<OscillationTracker>();
    switch (params->speed_mode) {
        case TripleController::NOT_SPEED:
            if(distance){
                robot->setTranslationalPosition(0);
                robot->setTranslationalTarget(target);
                robot->setDoneDistance(false);
                computeIndex = robot->addEndComputeHooks([this](){
                    oscTracker->update(robot->getTranslationalTarget() - robot->getTranslationalPosition(), robot->getDT() * 1000 * 1000);
                    buffer->write_raw(robot->getTranslationalPosition());
                    buffer->write_raw(robot->getTranslationalTarget());
                    buffer->write_raw(robot->getTranslationalRampSpeed());
                    buffer->write_raw(robot->getTranslationalEstimatedSpeed());
                    buffer->write_raw(robot->getDT());
                    buffer->write_raw(robot->getLeftMotor()->getPWM());
                    buffer->write_raw(robot->getRightMotor()->getPWM());
                });
            }else{
                robot->setRotationalPosition(AngleConstants::ZERO);
                robot->setRotationalTarget(Angle::fromDegrees(target));
                robot->setDoneAngular(false);
                computeIndex = robot->addEndComputeHooks([this](){
                    oscTracker->update((robot->getRotationalTarget() - robot->getRotationalPosition()).toDegrees(), robot->getDT() * 1000 * 1000);
                    buffer->write_raw(robot->getRotationalPosition().toDegrees());
                    buffer->write_raw(robot->getRotationalTarget().toDegrees());
                    buffer->write_raw(robot->getRotationalRampSpeed().toDegrees());
                    buffer->write_raw(robot->getRotationalEstimatedSpeed().toDegrees());
                    buffer->write_raw(robot->getDT());
                    buffer->write_raw(robot->getLeftMotor()->getPWM());
                    buffer->write_raw(robot->getRightMotor()->getPWM());
               });
            }
            break;
        case TripleController::SPEED_DEFAULT:
            if(distance){
                robot->getDistanceEstimator()->reset();
                robot->setTranslationalRampSpeed(target);
                robot->setDoneDistance(false);
                oscTracker->set_oscillate_around_zero(true);
                computeIndex = robot->addEndComputeHooks([this](){
                    oscTracker->update(robot->getTranslationalRampSpeed() - robot->getTranslationalEstimatedSpeed(), robot->getDT() * 1000 * 1000);
                    buffer->write_raw(robot->getTranslationalPosition());
                    buffer->write_raw(robot->getTranslationalTarget());
                    buffer->write_raw(robot->getTranslationalRampSpeed());
                    buffer->write_raw(robot->getTranslationalEstimatedSpeed());
                    buffer->write_raw(robot->getDT());
                    buffer->write_raw(robot->getLeftMotor()->getPWM());
                    buffer->write_raw(robot->getRightMotor()->getPWM());
                });
            }else{
                robot->getAngleEstimator()->reset();
                robot->setRotationalRampSpeed(Angle::fromDegrees(target));
                robot->setDoneAngular(false);
                oscTracker->set_oscillate_around_zero(true);
                computeIndex = robot->addEndComputeHooks([this](){
                    oscTracker->update((robot->getRotationalRampSpeed() - robot->getRotationalEstimatedSpeed()).toDegrees(), robot->getDT() * 1000 * 1000);
                    buffer->write_raw(robot->getRotationalPosition().toDegrees());
                    buffer->write_raw(robot->getRotationalTarget().toDegrees());
                    buffer->write_raw(robot->getRotationalRampSpeed().toDegrees());
                    buffer->write_raw(robot->getRotationalEstimatedSpeed().toDegrees());
                    buffer->write_raw(robot->getDT());
                    buffer->write_raw(robot->getLeftMotor()->getPWM());
                    buffer->write_raw(robot->getRightMotor()->getPWM());
                });
            }
            break;
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
                    finalGain = this->pid->getKp();
                    finalPeriod = oscTracker->get_oscillation_period_s();
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
        switch (speed) {
            case TripleController::NOT_SPEED:
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
                break;
            case TripleController::SPEED_DEFAULT:
                oscTracker->set_oscillate_around_zero(true);
                if (distance) {
                    if (forward) {
                        robot->setTranslationalRampSpeed(-target);
                    }else {
                        robot->setTranslationalRampSpeed(target);
                    }
                    forward = !forward;
                }else {
                    if (forward) {
                        robot->setRotationalRampSpeed(Angle::fromDegrees(-target));
                    }else {
                        robot->setRotationalRampSpeed(Angle::fromDegrees(target));
                    }
                    forward = !forward;
                }
                break;
        }
        robot->setControlDisabled(false);
    }, seconds(10));
    robot->setControlDisabled(false);
}

void ZieglerNicholsMethodoTriplePID::save() {

}

ZieglerNicholsMethodoTriplePID::ZieglerNicholsMethodoTriplePID(std::shared_ptr<BaseRobot> robot, std::shared_ptr<Mutex> mutex, bool distance, TripleController::SpeedMode speed) : CalibrationMethodo(robot, mutex), distance(distance), speed(speed) {
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
    data += ".bin";
    CalibrationMethodo::openFile(data.c_str());
    switch (speed) {
        case TripleController::NOT_SPEED:
            if (distance) {
                buffer->write_raw(static_cast<uint64_t>(BinaryFileType::Z_N_LEGACY_DISTANCE));
            }else {
                buffer->write_raw(static_cast<uint64_t>(BinaryFileType::Z_N_LEGACY_ANGLE));
            }
            break;
        case TripleController::SPEED_DEFAULT:
            if (distance) {
                buffer->write_raw(static_cast<uint64_t>(BinaryFileType::Z_N_LEGACY_DISTANCE_SPEED));
            }else {
                buffer->write_raw(static_cast<uint64_t>(BinaryFileType::Z_N_LEGACY_ANGLE_SPEED));
            }
            break;
    }
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

void ZieglerNicholsMethodoTriplePID::setPIDTo(ZieglerPIDChoice::Choice choice) {
    auto controller = distance ? std::static_pointer_cast<SimpleTripleBasicController>(robot->getController())->getDistanceController() : std::static_pointer_cast<SimpleTripleBasicController>(robot->getController())->getAngleController();
    auto pid = BasicControllerDeserialisation::castToPID(controller);
    if (pid ==nullptr) {
        return;
    }
#define PID_CHOICE(name, p, i, d)\
    case ZieglerPIDChoice::name: \
        pid->setKP((p)*this->finalGain); \
        pid->setKI((i)*this->finalGain/this->finalPeriod);\
        pid->setKD((d)*this->finalGain*this->finalPeriod);\
        break;

    switch (choice) {
        ZIEGLER_NICHOLS_PID_CHOICE
    }
#undef PID_CHOICE
}
