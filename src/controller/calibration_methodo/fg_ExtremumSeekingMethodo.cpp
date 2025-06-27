//
// Created by fogoz on 05/06/2025.
//

#include <controller/calibration_methodo/ExtremumSeekingMethodo.h>

#include "basic_controller/BasicControllerFactory.h"
#include "basic_controller/PID.h"
#include "ramp/CalculatedQuadramp.h"
#include "target/AngleTarget.h"
#include "target/DistanceTarget.h"
#include "target/ContinuousCurveTarget.h"
#include "utils/G2Solve3Arc.h"
#include "curves/CurveList.h"
#include "ramp/DynamicQuadRamp.h"

void FLASHMEM ExtremumSeekingMethodo::launchStage() {
    initialKP = pid->getKp();
    initialKI = pid->getKi();
    initialKD = pid->getKd();
    time = 0;
    for (auto& iq : iqs) {
        iq.I = 0;
        iq.Q = 0;
    }
    if (distance == ESCType::DISTANCE) {
        COMPLETE_DISTANCE_TARGET(500, RampData(100,200));
        COMPLETE_DISTANCE_TARGET(-1000, RampData(100,200));
        COMPLETE_DISTANCE_TARGET(500, RampData(100,200));
    }else if(distance == ESCType::ANGLE){
        COMPLETE_ANGLE_TARGET(AngleConstants::LEFT, RampData(90,180));
        COMPLETE_ANGLE_TARGET(AngleConstants::RIGHT, RampData(90,180));
        COMPLETE_ANGLE_TARGET(AngleConstants::FRONT, RampData(90,180));
    }else if(distance == ESCType::DISTANCE_ANGLE){
        Position start = robot->getCurrentPosition();
        Position end(1000, -200, Angle::fromDegrees(45), 0);
        Position end2(1200, 400, Angle::fromDegrees(180), 0);
        Position end3(800, 400, Angle::fromDegrees(225), 0);
        Position end4(400, 0, Angle::fromDegrees(180), 0);
        Position end5(0,0, Angle::fromDegrees(180), 0);
        //Position end(1000, 100, Angle::fromDegrees(90), 0);
        G2Solve3Arc arc;
        auto curveList = arc.getCurveList();
        arc.build(start, end);
        curveList->addCurveList(arc.getCurveList());
        arc.build(end, end2);
        curveList->addCurveList(arc.getCurveList());
        arc.build(end2, end3);
        curveList->addCurveList(arc.getCurveList());
        arc.build(end3, end4);
        curveList->addCurveList(arc.getCurveList());
        arc.build(end4, end5);
        curveList->addCurveList(arc.getCurveList());
        target = std::make_shared<ContinuousCurveTarget<DynamicQuadRamp>>(robot, curveList, RampData(100,400));
        robot->addTarget(target);
        //COMPLETE_ANGLE_TARGET(Angle::fromDegrees(0), RampData(90,180));

    }
}

void FLASHMEM ExtremumSeekingMethodo::cleanupStage(std::function<void()> callback) {
    if (robot->getTargetCount() != 0)
        return;
    //pid->getKpRef() = initialKP - gammaKP * iqs[0].I;//sqrt(pow(iqs[0].I, 2) + pow(iqs[0].Q, 2));
    //pid->getKiRef() = initialKI - gammaKI * iqs[1].I; //sqrt(pow(iqs[1].I, 2) + pow(iqs[1].Q, 2));
    //pid->getKdRef() = initialKD - gammaKD * iqs[2].I; //sqrt(pow(iqs[2].I, 2) + pow(iqs[2].Q, 2));
#define UPDATE(ref, rest, i) \
    pid->get##ref##Ref() = initial##rest - gamma##rest * sqrt(pow(iqs[i].I, 2) + pow(iqs[i].Q, 2)) * cos(atan2(iqs[i].Q, iqs[i].I));

    UPDATE(Kp, KP, 0)
    UPDATE(Ki, KI, 1)
    UPDATE(Kd, KD, 2)
    //pid->getKpRef() = initialKP - gammaKP * sqrt(pow(iqs[0].I, 2) + pow(iqs[0].Q, 2));
    //pid->getKiRef() = initialKI - gammaKI * sqrt(pow(iqs[1].I, 2) + pow(iqs[1].Q, 2));
    //pid->getKdRef() = initialKD - gammaKD * sqrt(pow(iqs[2].I, 2) + pow(iqs[2].Q, 2));
    pid->getKpRef() = constrain(pid->getKpRef(), 1, 200);
    pid->getKiRef() = constrain(pid->getKiRef(), 0.0001, 1000);
    pid->getKdRef() = constrain(pid->getKdRef(), 0.0001, 200);
    robot->getLeftMotor()->setPWM(0);
    robot->getRightMotor()->setPWM(0);
    previousLeft = 0;
    previousRight = 0;
    streamSplitter.printf("Updating KP from %f to %f\r\nUpdating KI from %f to %f\r\nUpdating KD from %f to %f\r\n", initialKP, pid->getKpRef(), initialKI, pid->getKiRef(), initialKD, pid->getKdRef());
    tasksId.push_back(scheduler->addTask(milliseconds(100), [this, callback]() {
        if (callback != nullptr)
            callback();
    }));
}

ExtremumSeekingMethodo::ExtremumSeekingMethodo(const std::shared_ptr<PAMIRobot> &robot,
                                               const std::shared_ptr<Mutex> &sdMutex, ESCType::ESC distance): CalibrationMethodo(robot, sdMutex), robot(robot), distance(distance) {

}

void FLASHMEM ExtremumSeekingMethodo::save() {
}

void FLASHMEM ExtremumSeekingMethodo::printStatus(Stream &stream) {
}

void FLASHMEM ExtremumSeekingMethodo::start() {
    CalibrationMethodo::start();
    if (distance == ESCType::DISTANCE) {
        assert(BasicControllerDeserialisation::isTypeCastableTo(robot->getControllerDistance()->getType(), BasicControllerType::PID));
        pid = std::static_pointer_cast<PID>(robot->getControllerDistance());
    }else if(distance == ESCType::ANGLE){
        assert(BasicControllerDeserialisation::isTypeCastableTo(robot->getControllerAngle()->getType(), BasicControllerType::PID));
        pid = std::static_pointer_cast<PID>(robot->getControllerAngle());
    }else if(distance == ESCType::DISTANCE_ANGLE){
        assert(BasicControllerDeserialisation::isTypeCastableTo(robot->getControllerDistanceAngle()->getType(), BasicControllerType::PID));
        pid = std::static_pointer_cast<PID>(robot->getControllerDistanceAngle());
    }
    robot->clearTarget();

    endComputeHook = robot->addEndComputeHooks([this]() {
        if (robot->getTargetCount() == 0 || waiting_turn)
            return;
        double dt = robot->getDT();
        double error = 0;
        switch(distance){
            case ESCType::ANGLE:
                error = (robot->getRotationalTarget() - robot->getRotationalPosition()).toDegrees();
                break;
            case ESCType::DISTANCE:
                error= (robot->getTranslationalTarget() - robot->getTranslationalPosition());
                break;
            case ESCType::DISTANCE_ANGLE:
                error = (target->getTargetPosition() - robot->getCurrentPosition()).getDistance();
                break;
        }
        double Jt = ISE_DU_DT(error);

        filtered_Jt = lpf_alpha * Jt + (1.0 - lpf_alpha) * filtered_Jt;
        double Jt_acc = Jt - filtered_Jt;
        iqs[0].I += Jt_acc * cos(time * frequencyKP) * dt;
        iqs[0].Q += Jt_acc * sin(time * frequencyKP) * dt;

        iqs[1].I += Jt_acc * cos(time * frequencyKI) * dt;
        iqs[1].Q += Jt_acc * sin(time * frequencyKI) * dt;

        iqs[2].I += Jt_acc * cos(time * frequencyKD) * dt;
        iqs[2].Q += Jt_acc * sin(time * frequencyKD) * dt;
        time += dt;
        pid->getKpRef() = initialKP + alphaKP * initialKP* cos(time * frequencyKP);
        pid->getKiRef() = initialKI + alphaKI * initialKI* cos(time * frequencyKI);
        pid->getKdRef() = initialKD + alphaKD * initialKD *cos(time * frequencyKD);
        previousLeft = robot->getLeftMotor()->getPWM();
        previousRight = robot->getRightMotor()->getPWM();
        if(time > 20.0){
            robot->clearTarget();
        }
    });
    allTargetEndedHook = robot->addAllTargetEndedHooks([this]() {
        if (waiting_turn) {
            tasksId.push_back(scheduler->addTask(milliseconds (1000), [this](){
                launchStage();
                waiting_turn = false;
            }));
            return;
        }
        cleanupStage([this]() {
            if (distance == ESCType::DISTANCE_ANGLE) {
                tasksId.push_back(scheduler->addTask(milliseconds(100), [this] {
                    COMPLETE_ANGLE_TARGET_DEG(0, RampData(90,180));
                    waiting_turn = true;
                }));
                return;
            }
            tasksId.push_back(scheduler->addTask(milliseconds (1000), [this](){
                launchStage();
            }));
        });
    });
    launchStage();
    robot->setControlDisabled(false);

}

void FLASHMEM ExtremumSeekingMethodo::stop() {
    CalibrationMethodo::stop();
    cleanupStage(nullptr);
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
