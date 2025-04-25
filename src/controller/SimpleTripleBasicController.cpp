//
// Created by fogoz on 24/04/2025.
//

#include "controller/SimpleTripleBasicController.h"
#include "robot/BaseRobot.h"
void SimpleTripleBasicController::compute() {

    if(!robot->isDoneDistance()){
        //PID Distance + Distance angle
        double distanceResult = distanceController->evaluate(robot->getTranslationalTarget() - robot->getTranslationalPosition());
        double angleResult = robot->isDoneAngular() ? 0 : distanceAngleController->evaluate(robot->getRotationalTarget() - robot->getRotationalPosition());

        distanceResult = applyMaxMin(distanceResult, params.maxValueDistance);
        angleResult = applyMaxMin(angleResult, params.maxValueAngular);

        double leftPWM = distanceResult - angleResult;
        double rightPWM = distanceResult + angleResult;

        leftPWM = applySpeedMin(leftPWM, params.speed_min_l);
        rightPWM = applySpeedMin(rightPWM, params.speed_min_r);
        robot->getLeftMotor()->setPWM(leftPWM);
        robot->getRightMotor()->setPWM(rightPWM);
    }else if(!robot->isDoneAngular()){
        //PID Angle
        double angleResult = angleController->evaluate(robot->getRotationalTarget() - robot->getRotationalPosition());
        angleResult = applyMaxMin(angleResult, params.maxValueAngular);

        robot->getLeftMotor()->setPWM(applySpeedMin(-angleResult, params.speed_min_l));
        robot->getRightMotor()->setPWM(applySpeedMin(angleResult, params.speed_min_r));
    }else{
        robot->getLeftMotor()->setPWM(0);
        robot->getRightMotor()->setPWM(0);
    }

}

void SimpleTripleBasicController::reset(bool correct_error) {
    if(correct_error){
        if(!robot->isDoneDistance()) {
            distanceController->reset(robot->getTranslationalTarget() - robot->getTranslationalPosition());
            distanceAngleController->reset(robot->isDoneAngular() ? 0 : (robot->getRotationalTarget() - robot->getRotationalPosition()));
            angleController->reset();
        }else{
            distanceController->reset();
            distanceAngleController->reset();
            angleController->reset(robot->isDoneAngular() ? 0 : (robot->getRotationalTarget() - robot->getRotationalPosition()));
        }
    }else{
        angleController->reset();
        distanceAngleController->reset();
        distanceController->reset();
    }
}


SimpleTripleBasicController::SimpleTripleBasicController(const std::shared_ptr<BasicController> &distanceController,
                                                         const std::shared_ptr<BasicController> &distanceAngleController,
                                                         const std::shared_ptr<BasicController> &angleController,
                                                         const std::shared_ptr<BaseRobot> &robot,
                                                         const TripleBasicParameters &params) : distanceController(
        distanceController), distanceAngleController(distanceAngleController), angleController(angleController),
                                                                                                robot(robot),
                                                                                                params(params) {}
