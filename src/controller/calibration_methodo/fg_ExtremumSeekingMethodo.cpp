//
// Created by fogoz on 05/06/2025.
//

#include <controller/calibration_methodo/ExtremumSeekingMethodo.h>

#include "basic_controller/PID.h"
#include "ramp/CalculatedQuadramp.h"
#include "target/AngleTarget.h"
#include "target/DistanceTarget.h"

void FLASHMEM ExtremumSeekingMethodo::launchStage() {
    initialKP = pid->getKp();
    initialKI = pid->getKi();
    initialKD = pid->getKd();
    time = 0;
    for (auto& iq : iqs) {
        iq.I = 0;
        iq.Q = 0;
    }
    if (distance) {
        COMPLETE_DISTANCE_TARGET(500, RampData(100,200));
        COMPLETE_DISTANCE_TARGET(-1000, RampData(100,200));
        COMPLETE_DISTANCE_TARGET(500, RampData(100,200));
    }else {
        COMPLETE_ANGLE_TARGET(AngleConstants::LEFT, RampData(90,180));
        COMPLETE_ANGLE_TARGET(AngleConstants::RIGHT, RampData(90,180));
        COMPLETE_ANGLE_TARGET(AngleConstants::FRONT, RampData(90,180));
    }
}

void FLASHMEM ExtremumSeekingMethodo::cleanupStage() {
    pid->getKpRef() = initialKP - gammaKP * iqs[0].I;//sqrt(pow(iqs[0].I, 2) + pow(iqs[0].Q, 2));
    pid->getKiRef() = initialKI - gammaKI * iqs[1].I; //sqrt(pow(iqs[1].I, 2) + pow(iqs[1].Q, 2));
    pid->getKdRef() = initialKD - gammaKD * iqs[2].I; //sqrt(pow(iqs[2].I, 2) + pow(iqs[2].Q, 2));
    pid->getKpRef() = constrain(pid->getKpRef(), 5, 200);
    pid->getKiRef() = constrain(pid->getKiRef(), 5, 1000);
    pid->getKdRef() = constrain(pid->getKdRef(), 5, 200);
    robot->getLeftMotor()->setPWM(0);
    robot->getRightMotor()->setPWM(0);
    previousLeft = 0;
    previousRight = 0;
    threads.delay(100);
    streamSplitter.printf("Updating KP from %f to %f\r\nUpdating KI from %f to %f\r\nUpdating KD from %f to %f\r\n", initialKP, pid->getKp(), initialKI, pid->getKi(), initialKD, pid->getKd());
}

ExtremumSeekingMethodo::ExtremumSeekingMethodo(const std::shared_ptr<PAMIRobot> &robot,
                                               const std::shared_ptr<Mutex> &sdMutex, bool distance): CalibrationMethodo(robot, sdMutex), distance(distance) {
    this->robot = robot;
}

void FLASHMEM ExtremumSeekingMethodo::save() {
}

void FLASHMEM ExtremumSeekingMethodo::printStatus(Stream &stream) {
}

void FLASHMEM ExtremumSeekingMethodo::start() {
    CalibrationMethodo::start();
    if (distance) {
        assert(robot->getControllerDistance()->getType() == BasicControllerType::PID || robot->getControllerDistance()->getType() == BasicControllerType::PIDSpeedFeedForward);
        pid = std::static_pointer_cast<PID>(robot->getControllerDistance());
    }else {
        assert(robot->getControllerDistance()->getType() == BasicControllerType::PID || robot->getControllerDistance()->getType() == BasicControllerType::PIDSpeedFeedForward);
        pid = std::static_pointer_cast<PID>(robot->getControllerAngle());
    }
    robot->clearTarget();

    endComputeHook = robot->addEndComputeHooks([this]() {
        double dt = robot->getDT();
        time += dt;
        double error = distance ? (robot->getTranslationalTarget() - robot->getTranslationalPosition()) :(robot->getRotationalTarget() - robot->getRotationalPosition()).toDegrees();
        double Jt = ISE_DU_DT(error);
        iqs[0].I += Jt * cos(time * frequencyKP) * dt;
        iqs[0].Q += Jt * sin(time * frequencyKP) * dt;

        iqs[1].I += Jt * cos(time * frequencyKI) * dt;
        iqs[1].Q += Jt * sin(time * frequencyKI) * dt;

        iqs[2].I += Jt * cos(time * frequencyKD) * dt;
        iqs[2].Q += Jt * sin(time * frequencyKD) * dt;
        pid->getKpRef() = initialKP + alphaKP * cos(time * frequencyKP);
        pid->getKiRef() = initialKI + alphaKI * cos(time * frequencyKI);
        pid->getKdRef() = initialKD + alphaKD * cos(time * frequencyKD);
        previousLeft = robot->getLeftMotor()->getPWM();
        previousRight = robot->getRightMotor()->getPWM();
    });
    allTargetEndedHook = robot->addAllTargetEndedHooks([this]() {
        cleanupStage();
        launchStage();
    });
    launchStage();
    robot->setControlDisabled(false);

}

void FLASHMEM ExtremumSeekingMethodo::stop() {
    CalibrationMethodo::stop();
    cleanupStage();
    robot->removeAllTargetEndedHooks(allTargetEndedHook);
    robot->removeEndComputeHooks(endComputeHook);
}

void ExtremumSeekingMethodo::setAlphaKP(double alpha_kp) {
    alphaKP = alpha_kp;
}

void ExtremumSeekingMethodo::setAlphaKI(double alpha_ki) {
    alphaKI = alpha_ki;
}

void ExtremumSeekingMethodo::setAlphaKD(double alpha_kd) {
    alphaKD = alpha_kd;
}

void ExtremumSeekingMethodo::setGammaKP(double gamma_kp) {
    gammaKP = gamma_kp;
}

void ExtremumSeekingMethodo::setGammaKI(double gamma_ki) {
    gammaKI = gamma_ki;
}

void ExtremumSeekingMethodo::setGammaKD(double gamma_kd) {
    gammaKD = gamma_kd;
}

double FLASHMEM ExtremumSeekingMethodo::ITAE(double error) {
    return time * abs(error);
}

double FLASHMEM ExtremumSeekingMethodo::IAE(double error) {
    return abs(error);
}

double FLASHMEM ExtremumSeekingMethodo::ISE(double error) {
    return pow(error, 2);
}

double FLASHMEM ExtremumSeekingMethodo::ISE_DU_DT(double error) {
    double left = robot->getLeftMotor()->getPWM() - previousLeft;
    double right = robot->getRightMotor()->getPWM() - previousRight;
    return ISE(error) + lambda * (pow(left, 2) + pow(right, 2));
}

void ExtremumSeekingMethodo::setLambda(double lambda) {
    this->lambda = lambda;
}
