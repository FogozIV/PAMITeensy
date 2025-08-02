//
// Created by fogoz on 24/04/2025.
//
#include "robot/PAMIRobot.h"

#include <encoders/QuadEncoderImpl.h>
#include <motor/DirPWMMotor.h>

#include "basic_controller/BasicControllerFactory.h"
#include "basic_controller/PID.h"
#include "controller/ControllerFactory.h"
#include "utils/BufferFilePrint.h"

Matrix<6, 6> PAMIRobot::makeA() {
    return Matrix<6,6>({
        std::array<double, 6>{1.0,  dt,  0.0,    0.0,     0.0,       0.0},
        std::array<double, 6>{0.0, 0.0,  0.0,    0.0,     0.5,       0.5},
        std::array<double, 6>{0.0, 0.0,  1.0,    dt,      0.0,       0.0},
        std::array<double, 6>{0.0, 0.0,  0.0,    0.0, -1.0/positionManagerParameters->track_mm, 1.0/positionManagerParameters->track_mm},
        std::array<double, 6>{0.0, 0.0,  0.0,    0.0,     1.0,       0.0},
        std::array<double, 6>{0.0, 0.0,  0.0,    0.0,     0.0,       1.0}
    });
}

Matrix<2, 6> PAMIRobot::makeH() {
    return Matrix<2,6>({
        std::array<double, 6>{0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
        std::array<double, 6>{0.0, 0.0, 0.0, 0.0, 0.0, 1.0}
    });
}

Matrix<6,6> PAMIRobot::makeQ() {
    return Matrix<6,6>({
        std::array<double, 6>{1e-5, 0, 0, 0, 0, 0},
        std::array<double, 6>{0, 1e-4, 0, 0, 0, 0},
        std::array<double, 6>{0, 0, 1e-5, 0, 0, 0},
        std::array<double, 6>{0, 0, 0, 1e-4, 0, 0},
        std::array<double, 6>{0, 0, 0, 0, 1e-3, 0},
        std::array<double, 6>{0, 0, 0, 0, 0, 1e-3}
    });
}

Matrix<2, 2> PAMIRobot::makeR() {
    return Matrix<2,2> ({
        //4 tick de bruit
        std::array<double, 2>{pow(4*positionManagerParameters->left_wheel_diam, 2), 0.0},
        std::array<double, 2>{0.0, pow(4*positionManagerParameters->right_wheel_diam, 2)}
    });
}

double PAMIRobot::getState(KalmanFilter::KalmanStates state) {
    return kalmanFilter->getState(state);
}

PAMIRobot::PAMIRobot(std::shared_ptr<Mutex> motorUpdate) : BaseRobot(PAMIRobotType, motorUpdate) {
}

double PAMIRobot::getDT() {
    return dt;
}

void PAMIRobot::computeTarget() {
    targetMutex.lock();
    bool removed = false;
    if (!targets.empty()) {
        targets.front()->call_init();
        targets.front()->process();
        if (targets.front()->is_done()) {
            targets.front()->call_done();
            targets.pop();
            removed = true;
            if (!targets.empty()) {
                targets.front()->call_init();
                targets.front()->process();
            }
        }
    }
    bool empty = targets.empty();
    targetMutex.unlock();
    //We go out of the mutex just in case something discusting happens inside those hooks (which BTW must be lightweight to not compromise the main loop)
    if (removed) {
        TargetEndedHooks.call();
    }
    if (removed && empty) {
        AllTargetEndedHooks.call();
    }
}

void FASTRUN PAMIRobot::computePosition() {
    this->positionMutex.lock();
    Position init_pos = this->pos;
    auto [pos, distance, angle] = positionManager->computePosition(init_pos);
    auto [posWheel, distanceWheel, angleWheel] = motorWheelPositionManager->computePosition(this->motorPos);
    if (!control_disabled) {
        bufferPrinter->print("pos: ");
        bufferPrinter->print(pos);
        bufferPrinter->printf(", distance: %f, angle: %f\r\n", distance, angle);
    }
    this->pos = pos;
    this->positionMutex.unlock();
    this->motorPos = posWheel;
    this->translationPos += distance;
    this->rotationPos += Angle::fromRadians(angle);
}

void PAMIRobot::computeController() {
    controller->compute();
}

void PAMIRobot::addTarget(std::shared_ptr<BaseTarget> target) {
    targetMutex.lock();
    targets.push(target);
    targetMutex.unlock();
}

void PAMIRobot::compute() {
    std::chrono::duration<double, std::ratio<1, 1> > _dt{};
    auto current = std::chrono::steady_clock::now();
    _dt = current - previous_time;
    previous_time = current;
    dt = _dt.count();
    computePosition();
    if (!control_disabled) {
        computeTarget();
        if (!control_disabled) {
            computeController();
        }
        //Print the PLL values
        bufferPrinter->printf("PLL= %f; %f\r\n", distanceSpeedEstimator->getSpeed(), angleSpeedEstimator->getSpeed());
    }
    this->endOfComputeNotifier->notify();
    this->callEndComputeHooks();
}


void FLASHMEM PAMIRobot::init() {
    previous_time = std::chrono::steady_clock::now();
    leftEncoder = std::make_shared<QuadEncoderImpl>(0, 1, 1);
    rightEncoder = std::make_shared<QuadEncoderImpl>(2, 3, 2);
    leftWheelEncoder = std::make_shared<QuadEncoderImpl>(4, 7, 3);
    rightWheelEncoder = std::make_shared<QuadEncoderImpl>(8, 30, 4);
    ax12Handler = std::make_shared<AX12Handler>(Serial5, 1000000);
    std::shared_ptr<PAMIRobot> robot = shared_from_this();
    std::lock_guard guard(*sdMutex);
    bool sd_present = SD.begin(BUILTIN_SDCARD);
    bool filled = false;
    if (sd_present) {
        File data_file = SD.open("PAMIRobot.json", FILE_READ);
        if (data_file) {
            JsonDocument document;
            deserializeJson(document, data_file);
            if (document["motor"].is<JsonObject>()) {
                auto motor = document["motor"].as<JsonObject>();
                motorInversed = motor["inversed"].as<bool>();
                if (motor["left_motor"].is<MotorParameters>()) {
                    leftMotorParameters = std::make_shared<MotorParameters>(motor["left_motor"].as<MotorParameters>());
                } else {
                    leftMotorParameters = std::make_shared<MotorParameters>();
                }
                if (motor["right_motor"].is<MotorParameters>()) {
                    rightMotorParameters = std::make_shared<
                        MotorParameters>(motor["right_motor"].as<MotorParameters>());
                } else {
                    rightMotorParameters = std::make_shared<MotorParameters>();
                }
                leftMotor = std::make_shared<DirPWMMotor>(LEFT_PWM, LEFT_DIR, leftMotorParameters);
                rightMotor = std::make_shared<DirPWMMotor>(RIGHT_PWM, RIGHT_DIR, rightMotorParameters);
            } else {
                leftMotorParameters = std::make_shared<MotorParameters>();
                rightMotorParameters = std::make_shared<MotorParameters>();
                leftMotor = std::make_shared<DirPWMMotor>(LEFT_PWM, LEFT_DIR, leftMotorParameters);
                rightMotor = std::make_shared<DirPWMMotor>(RIGHT_PWM, RIGHT_DIR, rightMotorParameters);
            }
            if(document["controller"].is<JsonObject>()){
                controller = ControllerFactory::getFromJson(robot, document["controller"].as<JsonObject>());
            }else{
                controller = std::make_shared<SimpleTripleBasicController>(robot);
            }
            if (document["distance_estimator_bandwidth"].is<double>()) {
                distanceSpeedEstimator = std::make_shared<SpeedEstimator>(
                    robot, document["distance_estimator_bandwidth"].as<double>());
            } else {
                distanceSpeedEstimator = std::make_shared<SpeedEstimator>(robot, 80);
            }
            if (document["angle_estimator_bandwidth"].is<double>()) {
                angleSpeedEstimator = std::make_shared<SpeedEstimator>(
                    robot, document["angle_estimator_bandwidth"].as<double>());
            } else {
                angleSpeedEstimator = std::make_shared<SpeedEstimator>(robot, 80);
            }
            if (document["position_parameters"].is<PositionParameters>()) {
                positionManagerParameters = std::make_shared<PositionParameters>(
                    document["position_parameters"].as<PositionParameters>());
            } else {
                positionManagerParameters = std::make_shared<PositionParameters>();
            }
            if (document["position_parameters_wheel"].is<PositionParameters>()) {
                wheelPositionManagerParameters = std::make_shared<PositionParameters>(
                    document["position_parameters_wheel"].as<PositionParameters>());
            } else {
                wheelPositionManagerParameters = std::make_shared<PositionParameters>();
            }
            if (document["tolerances"].is<RobotTolerance>()) {
                tolerances = std::make_shared<RobotTolerance>(document["tolerances"].as<RobotTolerance>());
            }else {
                tolerances = std::make_shared<RobotTolerance>();
            }

            positionManager = std::make_shared<PositionManager>(robot, leftEncoder, rightEncoder,
                                                                positionManagerParameters, distanceSpeedEstimator,
                                                                angleSpeedEstimator, true);
            motorWheelPositionManager = std::make_shared<PositionManager>(
                robot, leftWheelEncoder, rightWheelEncoder, wheelPositionManagerParameters, wheelDistanceSpeedEstimator,
                wheelAngleSpeedEstimator);

            filled = true;
            data_file.close();
        }
    }
    if (!sd_present) {
        streamSplitter.println("WARNING=Please connect to SD card");
    }
    if (!filled) {
        streamSplitter.println("WARNING=Not filled putting default idiotic parameters");
        pinMode(LED_BUILTIN, OUTPUT);
        digitalWrite(LED_BUILTIN, HIGH);
        leftMotorParameters = std::make_shared<MotorParameters>();
        rightMotorParameters = std::make_shared<MotorParameters>();
        leftMotor = std::make_shared<DirPWMMotor>(LEFT_PWM, LEFT_DIR, leftMotorParameters);
        rightMotor = std::make_shared<DirPWMMotor>(RIGHT_PWM, RIGHT_DIR, rightMotorParameters);
        controller = std::make_shared<SimpleTripleBasicController>(robot);
        distanceSpeedEstimator = std::make_shared<SpeedEstimator>(robot, 80);
        angleSpeedEstimator = std::make_shared<SpeedEstimator>(robot, 80);
        positionManagerParameters = std::make_shared<PositionParameters>();
        wheelPositionManagerParameters = std::make_shared<PositionParameters>();
        positionManager = std::make_shared<PositionManager>(robot, leftEncoder, rightEncoder, positionManagerParameters,
                                                            distanceSpeedEstimator, angleSpeedEstimator, true);
        motorWheelPositionManager = std::make_shared<PositionManager>(robot, leftWheelEncoder, rightWheelEncoder,
                                                                      wheelPositionManagerParameters,
                                                                      wheelDistanceSpeedEstimator,
                                                                      wheelAngleSpeedEstimator);
        tolerances = std::make_shared<RobotTolerance>();
        motorInversed = false;
    }
    kalmanFilter = std::make_shared<KalmanFiltering<6,2>>(makeA(), makeH(), makeQ(), makeR());
}

bool FLASHMEM PAMIRobot::save() {
    return save("PAMIRobot.json");
}

bool FLASHMEM PAMIRobot::save(const char *filename) {
    sdMutex->lock();
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
    controller->serialize(document["controller"].to<JsonObject>());
    document["distance_estimator_bandwidth"] = distanceSpeedEstimator->getBandwidth();
    document["angle_estimator_bandwidth"] = angleSpeedEstimator->getBandwidth();
    document["position_parameters"] = (*positionManagerParameters);
    document["position_parameters_wheel"] = (*wheelPositionManagerParameters);
    document["tolerances"] = *tolerances;
    auto motor = document["motor"].to<JsonObject>();
    motor["inversed"] = motorInversed;
    motor["left_motor"] = (*leftMotorParameters);
    motor["right_motor"] = (*rightMotorParameters);
    serializeJson(document, data_file);
    data_file.flush();
    data_file.close();
    sdMutex->unlock();
    return true;
}

void PAMIRobot::reset_to(Position pos) {
    this->positionMutex.lock();
    this->pos = pos;
    this->motorPos = pos;
    this->positionMutex.unlock();
}

#define POSITION_PARAMS \
    POS_PARAM(left_wheel_diam)\
    POS_PARAM(right_wheel_diam)\
    POS_PARAM(track_mm)
#define POS_PARAM(name) \
parser.registerMathCommand(#name, this->positionManagerParameters->name, [](Stream& stream, double value, MathOP op){ \
    stream.printf("La valeur du paramètre "#name " est : %f\r\n", value);\
    return "";\
}, PSTR("change value or look at the value of parameter " #name));

#define MOTOR_PARAMS \
    MOTOR_PARAM(left)\
    MOTOR_PARAM(right)
#define MOTOR_PARAM(name) \
    MOTOR_P(name, resolution)\
    MOTOR_P(name, max_pwm)\
    MOTOR_P(name, inversed)

#define MOTOR_P(name, variable_name) \
parser.registerMathCommand("motor_"#name"_"#variable_name, this->name##MotorParameters->variable_name, [](Stream& stream, double value, MathOP op){ \
stream.printf("The value of the motor parameter "#variable_name"_"#name " is : %f\r\n", value);\
return "";\
}, PSTR("change value or look at the value of parameter motor_"#name"_"#variable_name));

#define PLL_PARAMS \
    PLL_PARAM(angle) \
    PLL_PARAM(distance)

#define PLL_PARAM(name) \
parser.registerMathCommand("pll_"#name, this->name##SpeedEstimator->getBandwidthRef(), [](Stream& stream, double value, MathOP op){ \
stream.printf("The value of the PLL parameter " #name " is : %f\r\n", value);\
return "";\
}, PSTR("change value or look at the value of parameter pll_" #name));

void FLASHMEM PAMIRobot::registerCommands(CommandParser &parser) {
    controller->registerCommands(parser, "");
    POSITION_PARAMS
    MOTOR_PARAMS
    PLL_PARAMS
}

void PAMIRobot::update(double left, double right) {
    kalmanFilter->update(makeA(), makeH(), makeQ(), makeR());
    kalmanFilter->computeWithMeasurement(Matrix<2,1>({std::array<double, 1>{left/dt}, std::array<double, 1>{right/dt}}));
    BaseRobot::update(left, right);
}

double PAMIRobot::getTranslationalOtherEstimatedSpeed() {
    //By default there is no other estimated speed
    //return getTranslationalEstimatedSpeed();
    return kalmanFilter->getState(KalmanFilter::KalmanStates::v);
}

Angle PAMIRobot::getRotationalOtherEstimatedSpeed() {
    //By default there is no other estimated speed
    //return getRotationalEstimatedSpeed();
    return Angle::fromRadians(kalmanFilter->getState(KalmanFilter::KalmanStates::omega));
}


#undef POSITION_PARAMS
#undef POS_PARAM
#undef MOTOR_P
#undef MOTOR_PARAM
#undef MOTOR_PARAMS
#undef PLL_PARAM
#undef PLL_PARAMS

void PAMIRobot::clearTarget() {
    targetMutex.lock();
    std::queue<std::shared_ptr<BaseTarget> > empty;
    swap(empty, targets);
    if (empty.size() != 0) {
        targets.push(empty.front());
        targets.front()->forceDone();
    }
    targetMutex.unlock();
}

size_t PAMIRobot::getTargetCount() {
    targetMutex.lock();
    size_t count = targets.size();
    targetMutex.unlock();
    return count;
}

void PAMIRobot::controllerClear() {
    this->controller->reset();
}
