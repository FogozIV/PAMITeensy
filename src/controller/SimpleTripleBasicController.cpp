//
// Created by fogoz on 24/04/2025.
//

#include "controller/SimpleTripleBasicController.h"
#include "robot/BaseRobot.h"
#include "utils/BufferFilePrint.h"

void SimpleTripleBasicController::compute() {
    robot->lockMotorMutex();
    if (!robot->isDoneDistance()) {
        //PID Distance + Distance angle

        double distanceResult = distanceController->evaluate(targetTranslational() - positionTranslational());
            //robot->getTranslationalTarget() - robot->getTranslationalPosition());

        double angleResult = robot->isDoneAngular()
                                 ? 0
                                 : distanceAngleController->evaluate((targetRotational() - positionRotational()).toDegrees());
                                     //(robot->getRotationalTarget() - robot->getRotationalPosition()).toDegrees());

        //bufferPrinter->printf("Controller= %f; %f; %f; %f; %f; %f;", robot->getRotationalTarget().toDegrees(), robot->getRotationalPosition().toDegrees(), robot->getTranslationalPosition(),robot->getTranslationalTarget(),angleResult, distanceResult);
        distanceResult = applyMaxMin(distanceResult, params->maxValueDistance);
        angleResult = applyMaxMin(angleResult, params->maxValueDistanceAngle);
        //bufferPrinter->printf("%f; %f\r\n",angleResult, distanceResult);

        double leftPWM = distanceResult - angleResult;
        double rightPWM = distanceResult + angleResult;
        //bufferPrinter->printf("PWMs=%f; %f\r\n",leftPWM, rightPWM);

        leftPWM = applySpeedMin(leftPWM, params->speed_min_l);
        rightPWM = applySpeedMin(rightPWM, params->speed_min_r);
        robot->getLeftMotor()->setPWM(leftPWM);
        robot->getRightMotor()->setPWM(rightPWM);
    } else if (!robot->isDoneAngular()) {
        //PID Angle
        double angleResult = angleController->evaluate((targetRotational() - positionRotational()).toDegrees());
            //(robot->getRotationalTarget() - robot->getRotationalPosition()).toDegrees());
        //bufferPrinter->printf("Controller= %f; %f; %f;", robot->getRotationalTarget().toDegrees(), robot->getRotationalPosition().toDegrees(), angleResult);
        angleResult = applyMaxMin(angleResult, params->maxValueAngular);
        //bufferPrinter->printf("%f\r\n", angleResult);
        double leftPWM = applySpeedMin(-angleResult, params->speed_min_l);
        double rightPWM = applySpeedMin(angleResult, params->speed_min_r);
        //bufferPrinter->printf("PWMs= %f; %f\r\n", leftPWM, rightPWM);
        robot->getLeftMotor()->setPWM(leftPWM);
        robot->getRightMotor()->setPWM(rightPWM);
    } else {
        robot->getLeftMotor()->setPWM(0);
        robot->getRightMotor()->setPWM(0);
    }
    robot->unlockMotorMutex();
}



void SimpleTripleBasicController::reset(bool correct_error) {
    if (correct_error) {
        distanceController->reset(robot->isDoneDistance()
                                      ? 0
                                      : /*robot->getTranslationalTarget() - robot->getTranslationalPosition()*/(targetTranslational() - positionTranslational()));
        distanceAngleController->reset((robot->isDoneAngular()
                                            ? AngleConstants::ZERO
                                            : /*(robot->getRotationalTarget() - robot->getRotationalPosition())*/(targetRotational() - positionRotational())).
            toDegrees());
        angleController->reset();
    } else {
        angleController->reset();
        distanceAngleController->reset();
        distanceController->reset();
    }
}

std::shared_ptr<BasicController> SimpleTripleBasicController::getDistanceController() const {
    return distanceController;
}

std::shared_ptr<BasicController> SimpleTripleBasicController::getDistanceAngleController() const {
    return distanceAngleController;
}

std::shared_ptr<BasicController> SimpleTripleBasicController::getAngleController() const {
    return angleController;
}

std::shared_ptr<TripleBasicParameters> SimpleTripleBasicController::getParameters() const {
    return params;
}

void SimpleTripleBasicController::serialize(JsonObject json) {
    SET_JSON_ADVANCED(speed_min_l, getParameters());
    SET_JSON_ADVANCED(speed_min_r, getParameters());
    SET_JSON_ADVANCED(maxValueDistance, getParameters());
    SET_JSON_ADVANCED(maxValueAngular, getParameters());
    SET_JSON_ADVANCED(maxValueDistanceAngle, getParameters());
    SET_JSON_ADVANCED(speed_mode, getParameters());
    SET_JSON_ADVANCED(type, this);
}


std::shared_ptr<BaseController> SimpleTripleBasicController::deserialize(std::shared_ptr<BaseRobot> robot,
JsonVariant &json) {
}

void SimpleTripleBasicController::registerCommands(CommandParser &parser, const char *name) {
    distanceController->registerCommands(parser, "distance");
    distanceAngleController->registerCommands(parser, "distance_angle");
    angleController->registerCommands(parser, "angle");
}

void SimpleTripleBasicController::unregisterCommands(CommandParser &parser, const char *name) {
    distanceController->unregisterCommands(parser, "distance");
    distanceAngleController->unregisterCommands(parser, "distance_angle");
    angleController->unregisterCommands(parser, "angle");
}


SimpleTripleBasicController::SimpleTripleBasicController(const std::shared_ptr<BaseRobot> &robot,
                                                         const std::shared_ptr<BasicController> &distanceController,
                                                         const std::shared_ptr<BasicController> &
                                                         distanceAngleController,
                                                         const std::shared_ptr<BasicController> &angleController,
                                                         const std::shared_ptr<TripleBasicParameters> &params,
                                                         const std::function<double()> &targetTranslational,
                                                         const std::function<double()> &positionTranslational,
                                                         const std::function<Angle()> &targetRotational,
                                                         const std::function<Angle()> &positionRotational):distanceController(distanceController),
                                                                                                           distanceAngleController(
                                                                                                               distanceAngleController),
                                                                                                           angleController(angleController),
                                                                                                           robot(robot), params(params),
                                                                                                           targetTranslational(targetTranslational),
                                                                                                           positionTranslational(
                                                                                                               positionTranslational),
                                                                                                           targetRotational(targetRotational),
                                                                                                           positionRotational(positionRotational){
    type = ControllerFactory::TRIPLE_BASIC;
}

SimpleTripleBasicController::SimpleTripleBasicController(const std::shared_ptr<BaseRobot> &robot,
    const std::shared_ptr<BasicController> &distanceController,
    const std::shared_ptr<BasicController> &distanceAngleController,
    const std::shared_ptr<BasicController> &angleController, const std::shared_ptr<TripleBasicParameters> &params){
    type = ControllerFactory::TRIPLE_BASIC;
    this->robot = robot;
    this->distanceController = distanceController;
    this->distanceAngleController = distanceAngleController;
    this->angleController = angleController;
    this->params = params;
    switch (this->params->speed_mode) {
        case TripleController::NOT_SPEED:
            this->targetTranslational = CATCH_IN_LAMBDA_MEMBER(this->robot, getTranslationalTarget);
            this->targetRotational =CATCH_IN_LAMBDA_MEMBER(this->robot, getRotationalTarget);
            this->positionTranslational = CATCH_IN_LAMBDA_MEMBER(this->robot, getTranslationalPosition);
            this->positionRotational = CATCH_IN_LAMBDA_MEMBER(this->robot, getRotationalPosition);
            break;
        case TripleController::SPEED_DEFAULT:
            this->targetTranslational = CATCH_IN_LAMBDA_MEMBER(this->robot, getTranslationalRampSpeed);
            this->targetRotational =CATCH_IN_LAMBDA_MEMBER(this->robot, getRotationalRampSpeed);
            this->positionTranslational = CATCH_IN_LAMBDA_MEMBER(this->robot, getTranslationalEstimatedSpeed);
            this->positionRotational = CATCH_IN_LAMBDA_MEMBER(this->robot, getRotationalEstimatedSpeed);
            break;
    }
}
