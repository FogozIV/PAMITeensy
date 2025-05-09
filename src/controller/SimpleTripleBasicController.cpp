//
// Created by fogoz on 24/04/2025.
//

#include "controller/SimpleTripleBasicController.h"
#include "robot/BaseRobot.h"
#include "utils/BufferFilePrint.h"

void SimpleTripleBasicController::compute() {

    if(!robot->isDoneDistance()){
        //PID Distance + Distance angle
        double distanceResult = distanceController->evaluate(robot->getTranslationalTarget() - robot->getTranslationalPosition());
        double angleResult = robot->isDoneAngular() ? 0 : distanceAngleController->evaluate((robot->getRotationalTarget() - robot->getRotationalPosition()).toDegrees());

        distanceResult = applyMaxMin(distanceResult, params->maxValueDistance);
        angleResult = applyMaxMin(angleResult, params->maxValueDistanceAngle);

        double leftPWM = distanceResult - angleResult;
        double rightPWM = distanceResult + angleResult;

        leftPWM = applySpeedMin(leftPWM, params->speed_min_l);
        rightPWM = applySpeedMin(rightPWM, params->speed_min_r);
        robot->getLeftMotor()->setPWM(leftPWM);
        robot->getRightMotor()->setPWM(rightPWM);
    }else if(!robot->isDoneAngular()){
        //PID Angle
        double angleResult = angleController->evaluate((robot->getRotationalTarget() - robot->getRotationalPosition()).toDegrees());
        bufferPrinter->printf("Controller= %f; %f; %f;", robot->getRotationalTarget().toDegrees(), robot->getRotationalPosition().toDegrees(), angleResult);
        angleResult = applyMaxMin(angleResult, params->maxValueAngular);
        bufferPrinter->printf("%f\r\n", angleResult);
        double leftPWM = applySpeedMin(-angleResult, params->speed_min_l);
        double rightPWM = applySpeedMin(angleResult, params->speed_min_r);
        bufferPrinter->printf("PWMs= %f; %f\r\n", leftPWM, rightPWM);
        robot->getLeftMotor()->setPWM(leftPWM);
        robot->getRightMotor()->setPWM(rightPWM);
    }else{
        robot->getLeftMotor()->setPWM(0);
        robot->getRightMotor()->setPWM(0);
    }

}

void SimpleTripleBasicController::reset(bool correct_error) {
    if(correct_error){
        distanceController->reset(robot->isDoneDistance() ? 0 : robot->getTranslationalTarget() - robot->getTranslationalPosition());
        distanceAngleController->reset((robot->isDoneAngular() ? AngleConstants::ZERO : (robot->getRotationalTarget() - robot->getRotationalPosition())).toDegrees());
        angleController->reset();
    }else{
        angleController->reset();
        distanceAngleController->reset();
        distanceController->reset();
    }
}


SimpleTripleBasicController::SimpleTripleBasicController(
        const std::shared_ptr<BaseRobot> &robot,
        const std::shared_ptr<BasicController> &distanceController,
        const std::shared_ptr<BasicController> &distanceAngleController,
        const std::shared_ptr<BasicController> &angleController,
        const std::shared_ptr<TripleBasicParameters> &params) : distanceController(distanceController), distanceAngleController(distanceAngleController), angleController(angleController),
                                                                                                robot(robot),
                                                                                                params(params) {}
