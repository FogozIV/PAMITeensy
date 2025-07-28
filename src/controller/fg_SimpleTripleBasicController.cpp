//
// Created by fogoz on 24/04/2025.
//

#include "controller/SimpleTripleBasicController.h"
#include "robot/BaseRobot.h"
#include "utils/BufferFilePrint.h"
#include "basic_controller/BasicControllerFactory.h"
#include "basic_controller/PIDFilteredD.h"
#include "basic_controller/PIDSpeedFeedForward.h"

void FASTRUN SimpleTripleBasicController::compute() {
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
    return SimpleTripleBasicController::deserialize_as_T<SimpleTripleBasicController>(robot, json);
}
#define CHANGE_CONTROLLER(name, fft) \
switch(args[0].asUInt64() + 1){ /* To ignore BasicController*/\
case BasicControllerType::PID:\
stream.println("Changing type of controller to PID"); \
this->set##name##Controller(std::make_shared<PID>(robot, BasicControllerDeserialisation::castToPID(get##name##Controller()))); \
break;\
case BasicControllerType::PIDSpeedFeedForward:\
stream.println("Changing type of controller to PID Feed forward");\
set##name##Controller(std::make_shared<PIDSpeedFeedForward>(robot, BasicControllerDeserialisation::castToPID(get##name##Controller()))); \
break;\
case BasicControllerType::PIDFilteredD:\
stream.println("Changing type of controller to PID filtered");\
set##name##Controller(std::make_shared<PIDFilteredD>(robot, BasicControllerDeserialisation::castToPID(get##name##Controller()))); \
break;\
case BasicControllerType::FeedForward:\
stream.println("Changing type of controller to Feed Forward wrapper");\
set##name##Controller(std::make_shared<FeedForward>(robot, get##name##Controller(), 1, FeedForwardType::fft)); \
break;\
default:\
stream.printf("Unknown type %u\r\n", args[0].asUInt64());\
break;\
}
#define TEXT_CONTROLLER(name)\
"Allows to change the controller "#name "to 0 = PID, 1= PID Feed Forward 2= PID Filtered D 3= Feed Forward wrapper"
void SimpleTripleBasicController::registerCommands(CommandParser &parser, const char *name) {
    distanceController->registerCommands(parser, "distance");
    distanceAngleController->registerCommands(parser, "distance_angle");
    angleController->registerCommands(parser, "angle");

    parser.registerCommand("change_distance_to", "u", [this](std::vector<CommandParser::Argument> args, Stream& stream){
        CHANGE_CONTROLLER(Distance, DISTANCE)
        return "";
    }, TEXT_CONTROLLER(distance));


    parser.registerCommand("change_angle_to", "u", [this](std::vector<CommandParser::Argument> args, Stream& stream){
        CHANGE_CONTROLLER(Angle, ANGLE);
        return "";
    }, TEXT_CONTROLLER(angle));
    parser.registerCommand("change_distance_angle_to", "u", [this](std::vector<CommandParser::Argument> args, Stream& stream){
        CHANGE_CONTROLLER(DistanceAngle, ANGLE);
        return "";
    }, TEXT_CONTROLLER(distance angle));

    parser.registerCommand("transfer_angular_pid", "d", [this](std::vector<CommandParser::Argument> args, Stream& stream) {
        DynamicJsonDocument doc(1024);
        JsonVariant variant = doc.to<JsonObject>();
        getAngleController()->serialize(variant);
        setDistanceAngleController(BasicControllerDeserialisation::getFromJson(robot, variant));
        getDistanceAngleController()->multiply(args[0].asDouble());
        return "PID transfered";
    });

}

void SimpleTripleBasicController::unregisterCommands(CommandParser &parser, const char *name) {
    distanceController->unregisterCommands(parser, "distance");
    distanceAngleController->unregisterCommands(parser, "distance_angle");
    angleController->unregisterCommands(parser, "angle");
    parser.removeCommand("change_distance_angle_to");
    parser.removeCommand("change_distance_to");
    parser.removeCommand("change_angle_to");
    parser.removeCommand("transfer_angular_pid");

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
    computeFromSpeedMode();
}

SimpleTripleBasicController::SimpleTripleBasicController(const std::shared_ptr<BaseRobot> &robot) {
    this->robot = robot;
    this->params = std::make_shared<TripleBasicParameters>();
    this->distanceController = std::make_shared<PID>(robot, 20, 0, 0, 1000);
    this->angleController = std::make_shared<PID>(robot, 20, 0, 0, 1000);
    this->distanceAngleController = std::make_shared<PID>(robot, 20, 0, 0, 1000);
}

void SimpleTripleBasicController::computeFromSpeedMode() {
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

void SimpleTripleBasicController::setParameters(const std::shared_ptr<TripleBasicParameters> &parameters) {
    this->params = parameters;
}
#define REGISTER_UNREGISTER_COMMAND(name) \
    this->name##Controller->unregisterCommands(parser, #name); \
    this->name##Controller->unregisterCommands(xbeeCommandParser, #name);\
    this->name##Controller = name##Controller; \
    name##Controller->registerCommands(parser, #name);\
    name##Controller->registerCommands(xbeeCommandParser, #name);


void SimpleTripleBasicController::setDistanceController(const std::shared_ptr<BasicController> &distanceController) {
    REGISTER_UNREGISTER_COMMAND(distance);
}

void SimpleTripleBasicController::setDistanceAngleController(
        const std::shared_ptr<BasicController> &distanceAngleController) {
    REGISTER_UNREGISTER_COMMAND(distanceAngle);
}

void SimpleTripleBasicController::setAngleController(const std::shared_ptr<BasicController> &angleController) {
    REGISTER_UNREGISTER_COMMAND(angle);
}


