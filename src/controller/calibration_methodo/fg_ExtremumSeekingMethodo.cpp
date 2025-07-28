//
// Created by fogoz on 05/06/2025.
//

#include <controller/calibration_methodo/ExtremumSeekingMethodo.h>

#include "basic_controller/BasicController.h"
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
    initialGains = controller->getGains();
    time = 0;
    iqs = {};
    filtered_Jt = 0.0;
    for (size_t size = 0; size < initialGains.size(); size++) {
        iqs.emplace_back(0.0, 0.0);
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

void FLASHMEM ExtremumSeekingMethodo::cleanupStage(std::function<void()> callback, bool update) {
    if (robot->getTargetCount() != 0)
        return;
    //pid->getKpRef() = initialKP - gammaKP * iqs[0].I;//sqrt(pow(iqs[0].I, 2) + pow(iqs[0].Q, 2));
    //pid->getKiRef() = initialKI - gammaKI * iqs[1].I; //sqrt(pow(iqs[1].I, 2) + pow(iqs[1].Q, 2));
    //pid->getKdRef() = initialKD - gammaKD * iqs[2].I; //sqrt(pow(iqs[2].I, 2) + pow(iqs[2].Q, 2));
    std::vector<double> results;
    if (update && !done) {
        for (size_t i = 0; i < iqs.size(); i++) {
            streamSplitter.printf("I; Q : %f; %f\r\n", iqs[i].first, iqs[i].second);
            streamSplitter.printf("COS %f, atan %f\r\n", cos(atan2(iqs[i].second, iqs[i].first)) * 180/M_PI, atan2(iqs[i].second, iqs[i].first) * 180/M_PI);
            results.emplace_back(sqrt(pow(iqs[i].first, 2) + pow(iqs[i].second, 2)) * cos(atan2(iqs[i].second, iqs[i].first)));
        }
        controller->final_update(initialGains, results);
    }
    robot->getController()->reset();
    robot->getLeftMotor()->setPWM(0);
    robot->getRightMotor()->setPWM(0);
    previousLeft = 0;
    previousRight = 0;
    tasksId.push_back(scheduler->addTask(milliseconds(100), [callback]() {
        if (callback != nullptr)
            callback();
    }));
}

ExtremumSeekingMethodo::ExtremumSeekingMethodo(const std::shared_ptr<PAMIRobot> &robot,
                                               const std::shared_ptr<Mutex> &sdMutex, ESCType::ESC distance, double gamma, double alpha): CalibrationMethodo(robot, sdMutex), robot(robot), distance(distance), gamma(gamma), alpha(alpha) {
    assert(robot->getController()->getType() == ControllerFactory::TRIPLE_BASIC);
}

void FLASHMEM ExtremumSeekingMethodo::save() {
}

void FLASHMEM ExtremumSeekingMethodo::printStatus(Stream &stream) {
}

void FLASHMEM ExtremumSeekingMethodo::start() {
    CalibrationMethodo::start();
    auto c = std::static_pointer_cast<SimpleTripleBasicController>(robot->getController());
    if (distance == ESCType::DISTANCE) {
        controller = c->getDistanceController();
        for (auto& gamma : controller->gamma) {
            gamma = this->gamma == -1 ? 0.01 : this->gamma;
        }
        for (auto& alpha : controller->alpha) {
            alpha = this->alpha == -1 ? 0.05 : this->alpha;
        }
    }else if(distance == ESCType::ANGLE){
        controller = c->getAngleController();
        for (auto& gamma : controller->gamma) {
            gamma = this->gamma == -1 ? 0.01 : this->gamma;
        }
        for (auto& alpha : controller->alpha) {
            alpha = this->alpha == -1 ? 0.05 : this->alpha;
        }
    }else if(distance == ESCType::DISTANCE_ANGLE){
        controller = c->getDistanceAngleController();
        for (auto& gamma : controller->gamma) {
            gamma = this->gamma == -1 ? 0.01 : this->gamma;
        }
        for (auto& alpha : controller->alpha) {
            alpha = this->alpha == -1 ? 0.05 : this->alpha;
        }
    }else {
        streamSplitter.println("Error detected");
        return;
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
        auto data = controller->update_gains(initialGains, time);
        for (size_t i = 0; i < data.size(); i++) {
            iqs[i].first += Jt_acc * data[i].first * dt;
            iqs[i].second += Jt_acc * data[i].second * dt;
        }
        time += dt;
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
    robot->clearTarget();
    cleanupStage(nullptr);
    robot->removeAllTargetEndedHooks(allTargetEndedHook);
    robot->removeEndComputeHooks(endComputeHook);
    robot->getEventEndOfComputeNotifier()->wait();
    robot->clearTarget();
    controller->setGains(initialGains);
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
