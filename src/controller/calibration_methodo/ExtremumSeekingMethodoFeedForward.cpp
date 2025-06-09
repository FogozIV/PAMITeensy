//
// Created by fogoz on 05/06/2025.
//

#include <controller/calibration_methodo/ExtremumSeekingMethodoFeedForward.h>

#include "basic_controller/PID.h"
#include "ramp/CalculatedQuadramp.h"
#include "target/AngleTarget.h"
#include "target/DistanceTarget.h"
#include "target/FunctionTarget.h"

void FLASHMEM ExtremumSeekingMethodoFeedForward::launchStage() {
    initialKP = pid->getKp();
    initialKI = pid->getKi();
    initialKD = pid->getKd();
    initialKPP = pid->getFeedForwardRef();
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

void FLASHMEM ExtremumSeekingMethodoFeedForward::cleanupStage() {
    pid->getKpRef() = initialKP - gammaKP * iqs[0].I;//sqrt(pow(iqs[0].I, 2) + pow(iqs[0].Q, 2));
    pid->getKiRef() = initialKI - gammaKI * iqs[1].I; //sqrt(pow(iqs[1].I, 2) + pow(iqs[1].Q, 2));
    pid->getKdRef() = initialKD - gammaKD * iqs[2].I; //sqrt(pow(iqs[2].I, 2) + pow(iqs[2].Q, 2));
    pid->getFeedForwardRef() = initialKPP - gammaKPP * iqs[3].I;
    pid->getKpRef() = constrain(pid->getKpRef(), 0.1, 400);
    pid->getKiRef() = constrain(pid->getKiRef(), 0.1, 1000);
    pid->getKdRef() = constrain(pid->getKdRef(), 0.1, 400);
    pid->getFeedForwardRef() = constrain(pid->getFeedForwardRef(), 0.1, 100);
    robot->getLeftMotor()->setPWM(0);
    robot->getRightMotor()->setPWM(0);
    previousLeft = 0;
    previousRight = 0;
    threads.delay(100);
    streamSplitter.printf("Updating KP from %f to %f\r\nUpdating KI from %f to %f\r\nUpdating KD from %f to %f\r\nUpdating KPP from %f to %f\r\n", initialKP, pid->getKp(), initialKI, pid->getKi(), initialKD, pid->getKd(), initialKPP, pid->getFeedForwardRef());
}

ExtremumSeekingMethodoFeedForward::ExtremumSeekingMethodoFeedForward(const std::shared_ptr<PAMIRobot> &robot,
                                               const std::shared_ptr<Mutex> &sdMutex, bool distance): CalibrationMethodo(robot, sdMutex), distance(distance) {
    this->robot = robot;
}

void FLASHMEM ExtremumSeekingMethodoFeedForward::save() {
}

void FLASHMEM ExtremumSeekingMethodoFeedForward::printStatus(Stream &stream) {
}

void FLASHMEM ExtremumSeekingMethodoFeedForward::start() {
    CalibrationMethodo::start();
    if (distance) {
        auto controller = robot->getControllerDistance();
        assert(controller->getType() == BasicControllerType::PID || controller->getType() == BasicControllerType::PIDSpeedFeedForward);
        pid = PIDSpeedFeedForward::fromPID(std::static_pointer_cast<PID>(robot->getControllerDistance()), 0.01, robot, PIDSpeedFeedForwardType::FeedForward::DISTANCE);
        this->previousController = robot->getControllerDistance();
        robot->setControllerDistance(pid);
    }else {
        auto controller = robot->getControllerAngle();
        assert(controller->getType() == BasicControllerType::PID || controller->getType() == BasicControllerType::PIDSpeedFeedForward);
        pid =  PIDSpeedFeedForward::fromPID(std::static_pointer_cast<PID>(robot->getControllerAngle()), 0.01, robot, PIDSpeedFeedForwardType::FeedForward::ANGLE);
        this->previousController = robot->getControllerAngle();
        robot->setControllerAngle(pid);
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

        iqs[3].I += Jt * cos(time * frequencyKPP) * dt;
        iqs[3].Q += Jt * sin(time * frequencyKPP) * dt;

        pid->getKpRef() = initialKP + alphaKP * cos(time * frequencyKP);
        pid->getKiRef() = initialKI + alphaKI * cos(time * frequencyKI);
        pid->getKdRef() = initialKD + alphaKD * cos(time * frequencyKD);
        pid->getFeedForwardRef() = initialKPP + alphaKPP * cos(time * frequencyKPP);
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

void FLASHMEM ExtremumSeekingMethodoFeedForward::stop() {
    CalibrationMethodo::stop();
    cleanupStage();
    robot->removeAllTargetEndedHooks(allTargetEndedHook);
    robot->removeEndComputeHooks(endComputeHook);
}

void ExtremumSeekingMethodoFeedForward::setAlphaKP(double alpha_kp) {
    alphaKP = alpha_kp;
}

void ExtremumSeekingMethodoFeedForward::setAlphaKI(double alpha_ki) {
    alphaKI = alpha_ki;
}

void ExtremumSeekingMethodoFeedForward::setAlphaKD(double alpha_kd) {
    alphaKD = alpha_kd;
}

void ExtremumSeekingMethodoFeedForward::setAlphaKPP(double alpha_kpp) {
    alphaKPP = alpha_kpp;
}

void ExtremumSeekingMethodoFeedForward::setGammaKP(double gamma_kp) {
    gammaKP = gamma_kp;
}

void ExtremumSeekingMethodoFeedForward::setGammaKI(double gamma_ki) {
    gammaKI = gamma_ki;
}

void ExtremumSeekingMethodoFeedForward::setGammaKD(double gamma_kd) {
    gammaKD = gamma_kd;
}

void ExtremumSeekingMethodoFeedForward::setGammaKPP(double gamma_kpp) {
    gammaKPP = gamma_kpp;
}

double FLASHMEM ExtremumSeekingMethodoFeedForward::ITAE(double error) {
    return time * abs(error);
}

double FLASHMEM ExtremumSeekingMethodoFeedForward::IAE(double error) {
    return abs(error);
}

double FLASHMEM ExtremumSeekingMethodoFeedForward::ISE(double error) {
    return pow(error, 2);
}

double FLASHMEM ExtremumSeekingMethodoFeedForward::ISE_DU_DT(double error) {
    double left = robot->getLeftMotor()->getPWM() - previousLeft;
    double right = robot->getRightMotor()->getPWM() - previousRight;
    return ISE(error) + lambda * (pow(left, 2) + pow(right, 2));
}

void ExtremumSeekingMethodoFeedForward::setLambda(double lambda) {
    this->lambda = lambda;
}
