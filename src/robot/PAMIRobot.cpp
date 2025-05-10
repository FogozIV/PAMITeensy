//
// Created by fogoz on 24/04/2025.
//

#include "robot/PAMIRobot.h"

#include <encoders/QuadEncoderImpl.h>
#include <motor/DirPWMMotor.h>

#include "basic_controller/PID.h"
#include "utils/BufferFilePrint.h"

double PAMIRobot::getDT() {
    return dt;
}

void PAMIRobot::computeTarget() {
    targetMutex.lock();
    if(!targets.empty()){
        targets.front()->call_init();
        targets.front()->process();
        if(targets.front()->is_done()){
            targets.front()->call_done();
            targets.pop_front();
        }
    }
    targetMutex.unlock();
}

void PAMIRobot::computePosition() {
    auto [pos, distance, angle] = positionManager->computePosition();
    if (!control_disabled) {
        bufferPrinter->print("pos: ");
        bufferPrinter->print(pos);
        bufferPrinter->printf(", distance: %f, angle: %f\r\n", pos, distance, angle);
    }
    this->pos = pos;
    this->translationPos += distance;
    this->rotationPos += Angle::fromRadians(angle);

}

void PAMIRobot::computeController() {
    controller->compute();
}

void PAMIRobot::addTarget(std::shared_ptr<BaseTarget> target) {
    targetMutex.lock();
    targets.push_back(target);
    targetMutex.unlock();
}

void PAMIRobot::compute() {
    std::chrono::duration<double, std::ratio<1, 1>> _dt{};
    auto current = std::chrono::steady_clock::now();
    _dt = current - previous_time;
    previous_time= current;
    dt = _dt.count();
    computePosition();
    if(!control_disabled){
        computeTarget();
        computeController();
        bufferPrinter->printf("PLL= %f; %f\r\n", getAngleEstimator()->getSpeed(), getDistanceEstimator()->getSpeed());
    }
}

void PAMIRobot::init() {
    previous_time = std::chrono::steady_clock::now();
    leftEncoder = std::make_shared<QuadEncoderImpl>(0,1,1);
    rightEncoder = std::make_shared<QuadEncoderImpl>(2,3,2);

    ax12Handler = std::make_shared<AX12Handler>(Serial2, 1000000);
    std::shared_ptr<PAMIRobot> robot = shared_from_this();

    bool sd_present = SD.begin(BUILTIN_SDCARD);
    bool filled = false;
    if(sd_present){
        File data_file = SD.open("PAMIRobot.json", FILE_READ);
        if (data_file) {
            JsonDocument document;
            deserializeJson(document, data_file);
            if (document["motor"].is<JsonObject>()) {
                auto motor = document["motor"].as<JsonObject>();
                motorInversed = motor["inversed"].as<bool>();
                if (motor["left_motor"].is<MotorParameters>()) {
                    leftMotorParameters = std::make_shared<MotorParameters>(motor["left_motor"].as<MotorParameters>());
                }else {
                    leftMotorParameters = std::make_shared<MotorParameters>();
                }
                if (motor["right_motor"].is<MotorParameters>()) {
                    rightMotorParameters = std::make_shared<MotorParameters>(motor["right_motor"].as<MotorParameters>());
                }else {
                    rightMotorParameters = std::make_shared<MotorParameters>();
                }
                leftMotor = std::make_shared<DirPWMMotor>(33, 34, leftMotorParameters);
                rightMotor = std::make_shared<DirPWMMotor>(36, 35, rightMotorParameters);
            }else {
                leftMotorParameters = std::make_shared<MotorParameters>();
                rightMotorParameters = std::make_shared<MotorParameters>();
                leftMotor = std::make_shared<DirPWMMotor>(33, 34, leftMotorParameters);
                rightMotor = std::make_shared<DirPWMMotor>(36, 35, rightMotorParameters);
            }
            if (document["pid_distance"].is<JsonObject>()) {
                pidDistance = getPIDFromJson(robot, document["pid_distance"].as<JsonObject>());
            }else {
                pidDistance = std::make_shared<PID>(robot, 20, 0,0, 1000);
            }
            if (document["pid_angle"].is<JsonObject>()) {
                pidAngle = getPIDFromJson(robot, document["pid_angle"].as<JsonObject>());
            }else {
                pidAngle = std::make_shared<PID>(robot, 20, 0,0, 1000);
            }
            if (document["pid_distance_angle"].is<JsonObject>()) {
                pidDistanceAngle = getPIDFromJson(robot, document["pid_distance_angle"].as<JsonObject>());
            }else {
                pidDistanceAngle = std::make_shared<PID>(robot, 20, 0,0, 1000);
            }
            if (document["triple_parameters"].is<TripleBasicParameters>()) {
                pidParameters = std::make_shared<TripleBasicParameters>(document["triple_parameters"].as<TripleBasicParameters>());
            }else {
                pidParameters = std::make_shared<TripleBasicParameters>();
            }

            controller = std::make_shared<SimpleTripleBasicController>(robot, pidDistance, pidDistanceAngle, pidAngle, pidParameters);

            if (document["distance_estimator_bandwidth"].is<double>()) {
                distanceSpeedEstimator = std::make_shared<SpeedEstimator>(robot, document["distance_estimator_bandwidth"].as<double>());
            }else {
                distanceSpeedEstimator = std::make_shared<SpeedEstimator>(robot, 80);
            }
            if (document["angle_estimator_bandwidth"].is<double>()) {
                angleSpeedEstimator = std::make_shared<SpeedEstimator>(robot, document["angle_estimator_bandwidth"].as<double>());
            }else {
                angleSpeedEstimator = std::make_shared<SpeedEstimator>(robot, 80);
            }
            if (document["position_parameters"].is<PositionParameters>()) {
                positionManagerParameters = std::make_shared<PositionParameters>(document["position_parameters"].as<PositionParameters>());
            }else {
                positionManagerParameters = std::make_shared<PositionParameters>();
            }

            positionManager = std::make_shared<PositionManager>(robot, leftEncoder, rightEncoder, positionManagerParameters);

            filled = true;
            data_file.close();
        }
    }
    if (!filled) {
        Serial.println("Not filled putting default idiotic parameters");
        leftMotorParameters = std::make_shared<MotorParameters>();
        rightMotorParameters = std::make_shared<MotorParameters>();
        leftMotor = std::make_shared<DirPWMMotor>(33, 34, leftMotorParameters);
        rightMotor = std::make_shared<DirPWMMotor>(36, 35, rightMotorParameters);
        pidDistance = std::make_shared<PID>(robot, 20, 0,0, 1000);
        pidAngle = std::make_shared<PID>(robot, 20, 0,0, 1000);
        pidDistanceAngle = std::make_shared<PID>(robot, 20, 0,0, 1000);
        pidParameters = std::make_shared<TripleBasicParameters>();
        controller = std::make_shared<SimpleTripleBasicController>(robot, pidDistance, pidDistanceAngle, pidAngle, pidParameters);
        distanceSpeedEstimator = std::make_shared<SpeedEstimator>(robot, 80);
        angleSpeedEstimator = std::make_shared<SpeedEstimator>(robot, 80);
        positionManagerParameters = std::make_shared<PositionParameters>();
        positionManager = std::make_shared<PositionManager>(robot, leftEncoder, rightEncoder, positionManagerParameters);
        motorInversed = false;
    }

}

bool PAMIRobot::save() {
    return save("PAMIRobot.json");
}

bool PAMIRobot::save(const char *filename) {
    bool sd_present = SD.begin(BUILTIN_SDCARD);
    if (!sd_present) {
        return false;
    }
    if (SD.exists(filename)) {
        SD.remove(filename);
    }
    File data_file = SD.open(filename, FILE_WRITE_BEGIN);
    if (!data_file) {
        return false;
    }
    JsonDocument document;
    PIDtoJson(pidDistance, document["pid_distance"].to<JsonObject>());
    PIDtoJson(pidAngle, document["pid_angle"].to<JsonObject>());
    PIDtoJson(pidDistanceAngle, document["pid_distance_angle"].to<JsonObject>());
    document["triple_parameters"] = (*pidParameters.get());
    document["distance_estimator_bandwidth"] = distanceSpeedEstimator->getBandwidth();
    document["angle_estimator_bandwidth"] = angleSpeedEstimator->getBandwidth();
    document["position_parameters"] = (*positionManagerParameters.get());
    auto motor = document["motor"].to<JsonObject>();
    motor["inversed"] = motorInversed;
    motor["left_motor"] = (*leftMotorParameters.get());
    motor["right_motor"] = (*rightMotorParameters.get());
    serializeJson(document, data_file);
    data_file.flush();
    data_file.close();
    return true;
}

void PAMIRobot::reset_to(Position pos) {
    this->pos = pos;
}
#define COMMANDS_PID \
COMMAND_PID(distance, this->pidDistance) \
COMMAND_PID(angle, this->pidAngle) \
COMMAND_PID(angle_distance, this->pidDistanceAngle)

#define COMMAND_PID(name, variable) \
SUB_COMMAND_PID(name, KP, (variable)->getKpRef())\
SUB_COMMAND_PID(name, KI, (variable)->getKiRef())\
SUB_COMMAND_PID(name, KD, (variable)->getKdRef())\
SUB_COMMAND_PID(name, anti_windup, (variable)->getAntiWindupRef())

#define SUB_COMMAND_PID(name, sub_name, variable) \
parser.registerMathCommand("pid_"#name"_"#sub_name, variable, [](Stream& stream, double value, MathOP op){ \
stream.printf("La valeur du PID "#name" "#sub_name" est : %f\r\n", value);\
return "";\
}, "change value or look at the value  of PID "#name" "#sub_name);

#define POSITION_PARAMS \
    POS_PARAM(left_wheel_diam)\
    POS_PARAM(right_wheel_diam)\
    POS_PARAM(track_mm)
#define POS_PARAM(name) \
parser.registerMathCommand(#name, this->positionManagerParameters->name, [](Stream& stream, double value, MathOP op){ \
    stream.printf("La valeur du paramètre "#name " est : %f\r\n", value);\
    return "";\
}, "change value or look at the value of parameter " #name);

#define MOTOR_PARAMS \
    MOTOR_PARAM(left)\
    MOTOR_PARAM(right)
#define MOTOR_PARAM(name) \
    MOTOR_P(name, resolution)\
    MOTOR_P(name, max_pwm)\
    MOTOR_P(name, inversed)

#define MOTOR_P(name, variable_name) \
parser.registerMathCommand("motor_"#name"_"#variable_name, this->name##MotorParameters->variable_name, [](Stream& stream, double value, MathOP op){ \
stream.printf("The value of the motor parameter " #variable_name"_"#name " is : %f\r\n", value);\
return "";\
}, "change value or look at the value of parameter motor_" #name"_"#variable_name);

#define PLL_PARAMS \
    PLL_PARAM(angle) \
    PLL_PARAM(distance)

#define PLL_PARAM(name) \
parser.registerMathCommand("pll_"#name, this->name##SpeedEstimator->getBandwidthRef(), [](Stream& stream, double value, MathOP op){ \
stream.printf("The value of the PLL parameter " #name " is : %f\r\n", value);\
return "";\
}, "change value or look at the value of parameter pll_" #name);

void PAMIRobot::registerCommands(CommandParser &parser) {
    COMMANDS_PID
    POSITION_PARAMS
    MOTOR_PARAMS
    PLL_PARAMS
}

#undef SUB_COMMAND_PID
#undef COMMAND_PID
#undef COMMANDS_PID
#undef POSITION_PARAMS
#undef POS_PARAM
#undef MOTOR_P
#undef MOTOR_PARAM
#undef MOTOR_PARAMS
#undef PLL_PARAM
#undef PLL_PARAMS
