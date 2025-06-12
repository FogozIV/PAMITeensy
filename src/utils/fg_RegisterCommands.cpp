//
// Created by fogoz on 13/05/2025.
//
#include "utils/RegisterCommands.h"

#include "FXUtil.h"
#include "controller/calibration_methodo/BenchmarkMethodo.h"
#include "controller/calibration_methodo/ExtremumSeekingMethodo.h"
#include "controller/calibration_methodo/ExtremumSeekingMethodoFeedForward.h"
#include "utils/InteractContext.h"
#include "controller/calibration_methodo/ZieglerNicholsMethodoTriplePID.h"
#include "ramp/CalculatedQuadramp.h"
#include "target/PositionTarget.h"
#include "TCPTeensyUpdater.h"
#include "target/RotateTowardTarget.h"
#include "controller/calibration_methodo/ClothoidBenchmark.h"

FLASHMEM void waitForMethodoStop(CalibrationMethodo* methodo, Stream& stream) {
    stream.printf("Use w, W, q, Q or space to stop the iterations\r\n");
    while (!methodo->isDone()) {
        while (stream.available()) {
            char c = stream.read();
            if(c == 'w' || c=='W' || c=='q' || c=='Q' || c == ' '){
                methodo->stop();
            }
        }
        threads.delay(50);
    }
}
FLASHMEM void registerCommands(CommandParser &parser, std::shared_ptr<BaseRobot> robot) {
    parser.registerCommand("hello", "", [](std::vector<CommandParser::Argument> arg, Stream& stream) {
        return "hello world !";
    }, PSTR("a test command that says hello world"));

    parser.registerCommand("help", "", [&parser](std::vector<CommandParser::Argument> arg, Stream& stream) {
        for (CommandParser::Command cmd: parser.command_definitions()) {
            stream.println((cmd.name + ": " + (cmd.description.length() == 0 ? "No description found" : cmd.description)).c_str());
        }
        return "";
    }, PSTR("a command that displays all the available commands"));

    parser.registerCommand("get_encoder", "", [robot](std::vector<CommandParser::Argument> arg, Stream& stream){
        stream.printf("Encoder left : %d, Encoder right: %d, Left Wheel Encoder %d, Right Wheel Encoder %d \r\n", robot->getLeftEncoderValue(), robot->getRightEncoderValue(), robot->getLeftWheelEncoderValue(), robot->getRightWheelEncoderValue());
        return "";
    });

    parser.registerCommand("encoder_calib_rotation_turn", "d", [robot](std::vector<CommandParser::Argument> arg, Stream& stream) {
        robot->endCalibrationAngleTurnEncoder(arg[0].asDouble());
        return "success";
    }, PSTR("a command that allows you to type in the number of turn the robot did and calibrate it automatically don't forget to call encoder_calib_init before calling this function"));

    parser.registerCommand("encoder_calib_init", "", [robot](std::vector<CommandParser::Argument> arg, Stream& stream) {
        robot->beginCalibrationEncoder();
        return "Started the encoder calibration please type encoder_calib_rotation_turn or encoder_calib_straight after moving the robot";
    }, PSTR("a command that allows you to start the calibration of the encoder"));

    parser.registerCommand("encoder_calib_integral_reset", "", [robot](std::vector<CommandParser::Argument> args, Stream& stream){
        robot->resetCalibrationEncoderList();
        robot->beginCalibrationEncoder();
        return "Reset & started the encoder calibration, please use encoder_calib_integral_save &/or encoder_calib_integral_straight, encoder_calib_integral_turn";
    });

    parser.registerCommand("encoder_calib_integral_save", "d", [robot](std::vector<CommandParser::Argument> args, Stream& stream){
        robot->addCalibrationData(args[0].asDouble());
        return "Saved the data point for later usage use encoder_calib_integral_turn or encoder_calib_integral_straight to finish the calibration process";
    });

    parser.registerCommand("encoder_calib_integral_init", "", [robot](std::vector<CommandParser::Argument> args, Stream& stream){
        robot->beginCalibrationEncoder();
        return "Started the encoder calibration please type encoder_calib_integral_save (multiple time by preference) & then call encoder_calib_integral_straight or encoder_calib_integral_turn";
    });

    parser.registerCommand("encoder_calib_integral_straight", "", [robot](std::vector<CommandParser::Argument> args, Stream& stream){
        robot->finalizeCalibrationForward();
        robot->printCalibrationParameters(stream);
        return "Done computing";
    }, "Perform the computation like every values where distances");

    parser.registerCommand("encoder_calib_integral_turn", "", [robot](std::vector<CommandParser::Argument> args, Stream& stream){
        robot->finalizeCalibrationRotation();
        robot->printCalibrationParameters(stream);
        return "Done computing";
    });

    parser.registerCommand("encoder_calib_straight", "d", [robot](std::vector<CommandParser::Argument> arg, Stream& stream) {
        robot->endCalibrationStraightEncoder(arg[0].asDouble());
        return "success";
    },PSTR("a command that allows you to type in the distance the robot did and calibrate it automatically don't forget to call encoder_calib_init before calling this function"));

    parser.registerCommand("calib_motors", "", [robot](std::vector<CommandParser::Argument> args, Stream& stream) {
        robot->calibrateMotors();
        return "success";
    }, PSTR("a command that allows you to correct the motor wiring internally"));

    parser.registerCommand("reset", "", [](std::vector<CommandParser::Argument> arg, Stream& stream) {
        REBOOT;
        while (true);
        return "if you see this there is an issue";
    }, PSTR("a command that allows you to reset the robot"));

    parser.registerCommand("test_ax12", "i", [robot](std::vector<CommandParser::Argument> arg, Stream& stream) {
        auto ax12 = robot->getAX12Handler()->get(arg[0].asInt64());
        std::vector<uint8_t> u = ax12.sendCommand({AX12_INSTRUCTION_PING});
        if (u.size() > 1) {
            stream.println("Success");
            printAX12Error(u[0], stream);
        }else {
            stream.println("Ping failed");
            if (u.size() > 0) {
                stream.println(u[0]);
            }
        }
        return "done";
    });
    parser.registerCommand("set_left_pwm", "d", [robot](std::vector<CommandParser::Argument> arg, Stream& stream){
        robot->getLeftMotor()->setPWM(arg[0].asDouble());
        return "success";
    }, PSTR("a command that allows you to set the pwm of the left motor"));
    parser.registerCommand("set_right_pwm", "d", [robot](std::vector<CommandParser::Argument> arg, Stream& stream){
        robot->getRightMotor()->setPWM(arg[0].asDouble());
        return "success";
    }, PSTR("a command that allows you to set the pwm of the right motor"));

    parser.registerCommand("disable_control", "i", [robot](std::vector<CommandParser::Argument> args, Stream& stream){
        robot->setControlDisabled(args[0].asInt64());
        return "success";
    }, PSTR("a command that allows you to disable the motor control"));

    //Interact command
    parser.registerCommand("interact", "", [robot](std::vector<CommandParser::Argument> args, Stream& stream){
        return enterInteractContext(robot, stream);
    });

    parser.registerCommand("position", "", [robot](std::vector<CommandParser::Argument> args, Stream& stream){
        stream.print("Robot current pos: ");
        robot->getCurrentPosition().printTo(stream);
        return " ";
    });

    parser.registerCommand("wheel_position", "", [robot](std::vector<CommandParser::Argument> args, Stream& stream) {
        robot->getMotorPosition().printTo(stream);
        return  " ";
    });

    parser.registerCommand("position_set", "ddd", [robot](std::vector<CommandParser::Argument> args, Stream& stream){
        robot->reset_to({args[0].asDouble(), args[1].asDouble(), Angle::fromDegrees(args[2].asDouble())});
        return PSTR("Position set successfully");
    });
    parser.registerCommand("position_reset", "", [robot](std::vector<CommandParser::Argument> args, Stream& stream){
        robot->reset_to({});
        return PSTR("Position set to (0,0,0)");
    });

    parser.registerCommand("flash", "", [](std::vector<CommandParser::Argument> args, Stream& stream) {
        stream.print("Beginning the flash : ");
        if (flashing_process) {
            return PSTR("unable to flash another process is already doing it");
        }
        pause_thread_info = true;
        flashing_process = true;
        uint32_t buffer_addr, buffer_size;
        stream.println();
        // create flash buffer to hold new firmware
        if (firmware_buffer_init( &buffer_addr, &buffer_size ) == 0) {
            stream.printf( "unable to create buffer\r\n" );
            stream.flush();
            flashing_process = false;
            return "error";
        }

        stream.printf( "created buffer = %1luK %s (%08lX - %08lX)\r\n",
                       buffer_size/1024, IN_FLASH(buffer_addr) ? "FLASH" : "RAM",
                       buffer_addr, buffer_addr + buffer_size );
        stream.println("READY for upload");
        // read hex file, write new firmware to flash, clean up, reboot
        update_firmware( &stream, &stream, buffer_addr, buffer_size, false);

        firmware_buffer_free(buffer_addr, buffer_size);
        flashing_process = false;
        pause_thread_info = false;

        return "";
    });

    parser.registerCommand("overrideEncoderToMotors", "i", [robot](std::vector<CommandParser::Argument> args, Stream& stream) {
        if (args[0].asInt64() == 1) {
            robot->setEncoderToMotors();
        }else {
            robot->setEncoderToFreeWheel();
        }
        return "Done";
    });

    parser.registerCommand("calibrate_wheel_encoder", "", [robot](std::vector<CommandParser::Argument> args, Stream& stream) {
        BaseRobot::calibrateMotorEncoder(stream, robot);
        return "Done";
    });

    parser.registerCommand("stop", "", [robot](std::vector<CommandParser::Argument> args, Stream& stream) {
        robot->setControlDisabled(true);
        robot->lockMotorMutex();
        robot->getLeftMotor()->setPWM(0);
        robot->getRightMotor()->setPWM(0);
        robot->unlockMotorMutex();
        return "";
    });

    parser.registerCommand("save", "", [robot](std::vector<CommandParser::Argument> args, Stream& stream) {
        robot->save();
        return PSTR("The configuration has been saved");
    });
    parser.registerCommand("rotateToward", "ddd", [robot](std::vector<CommandParser::Argument> args, Stream& stream) {
        robot->addTarget(std::make_shared<AngleTarget<CalculatedQuadramp>>(robot, Angle::fromDegrees(args[0].asDouble()), RampData(args[1].asDouble(), args[2].asDouble())));
        stream.println("Created target");
        return "";
    });

    parser.registerCommand("moveToward", "dddd", [robot](std::vector<CommandParser::Argument> args, Stream& stream) {
        robot->addTarget(std::make_shared<PositionTarget<CalculatedQuadramp>>(robot, Position(args[0].asDouble(), args[1].asDouble()), RampData(args[2].asDouble(), args[3].asDouble())));
        stream.println("Created target");
        return "";
    });

    parser.registerCommand("clearTarget", "", [robot](std::vector<CommandParser::Argument> args, Stream& stream){
        robot->clearTarget();
        stream.println("Cleared target");
        return "";
    });

    parser.registerCommand("enable_angle", "i", [robot](std::vector<CommandParser::Argument> args, Stream& stream){
        if(args[0].asInt64() != 0){
            robot->setRotationalTarget(robot->getRotationalPosition());
            robot->setDoneAngular(false);
            robot->setControlDisabled(false);
        }else{
            robot->setDoneAngular(true);
        }
        return "Done";
    });

    parser.registerCommand("enable_distance", "i", [robot](std::vector<CommandParser::Argument> args, Stream& stream){
        if(args[0].asInt64() != 0){
            robot->setTranslationalTarget(robot->getTranslationalPosition());
            robot->setDoneDistance(false);
            robot->setControlDisabled(false);
        }else{
            robot->setDoneDistance(true);
        }
        return "Done";
    });

    parser.registerCommand("target_curvilinear", "od", [robot](std::vector<CommandParser::Argument> args, Stream& stream) {
        if (args[0]) {
            robot->setTranslationalTarget(args[0].asDouble());
            stream.printf("Curvilinear target set to %f \r\n", robot->getTranslationalTarget());
        }else {
            stream.printf("Curvilinear target : %f\r\n", robot->getTranslationalTarget());
        }
        return "";
    });

    parser.registerCommand("position_curvilinear", "od", [robot](std::vector<CommandParser::Argument> args, Stream& stream) {
        if (args[0]) {
            robot->setTranslationalPosition(args[0].asDouble());
            stream.printf("Curvilinear target set to %f \r\n", robot->getTranslationalPosition());
        }else {
            stream.printf("Curvilinear target : %f\r\n", robot->getTranslationalPosition());
        }
        return "";
    });

    parser.registerCommand("target_angle", "od", [robot](std::vector<CommandParser::Argument> args, Stream& stream) {
        if (args[0]) {
            robot->setRotationalTarget(Angle::fromDegrees(args[0].asDouble()));
            stream.printf("Angle target set to %f \r\n", robot->getRotationalTarget().toDegrees());
        }else {
            stream.printf("Angle target : %f\r\n", robot->getRotationalTarget().toDegrees());
        }
        return "";
    });

    parser.registerCommand("position_angle", "od", [robot](std::vector<CommandParser::Argument> args, Stream& stream) {
        if (args[0]) {
            robot->setRotationalPosition(Angle::fromDegrees(args[0].asDouble()));
            stream.printf("Angle position set to %f \r\n", robot->getRotationalPosition().toDegrees());
        }else {
            stream.printf("Angle position : %f\r\n", robot->getRotationalPosition().toDegrees());
        }
        return "";
    });

    parser.registerCommand("target_count", "", [robot](std::vector<CommandParser::Argument> args, Stream& stream){
        stream.printf("There is %d \r\n", robot->getTargetCount());
        return "";
    });

    parser.registerCommand("sendToBluetooth", "s", [robot](std::vector<CommandParser::Argument> args, Stream& stream){
        Serial8.println(args[0].asString().c_str());
        return "";
    });

    parser.registerCommand("test_pwm_violent", "oi", [robot](std::vector<CommandParser::Argument> args, Stream & stream){
        robot->setControlDisabled(true);
        int i = 94;
        if(args[0]){
            i = args[0].asInt64();
        }
        for(; i < 4095; i+=200){
            robot->getLeftMotor()->setPWM(i);
            robot->getRightMotor()->setPWM(i);
            threads.delay(1000);
            robot->getLeftMotor()->setPWM(-i);
            robot->getRightMotor()->setPWM(-i);
            threads.delay(1000);
            robot->getLeftMotor()->setPWM(0);
            robot->getRightMotor()->setPWM(0);
            delay(1000);
            stream.printf("Tested %d\r\n",i);
        }
        return "done";
    });

    parser.registerCommand("test_pwm", "oi", [robot](std::vector<CommandParser::Argument> args, Stream& stream){
        robot->setControlDisabled(true);
        int i = 94;
        if(args[0]){
            i = args[0].asInt64();
        }
        for(; i < 4095; i+=200){
            robot->getLeftMotor()->setPWM(i);
            robot->getRightMotor()->setPWM(i);
            threads.delay(1000);
            robot->getLeftMotor()->setPWM(0);
            robot->getRightMotor()->setPWM(0);
            threads.delay(1000);
            stream.printf("Tested %d\r\n",i);
        }
        robot->getLeftMotor()->setPWM(4095);
        robot->getRightMotor()->setPWM(4095);
        threads.delay(1000);
        robot->getLeftMotor()->setPWM(0);
        robot->getRightMotor()->setPWM(0);
        return "Done iteration";
    });

    parser.registerCommand("crash", "", [robot](std::vector<CommandParser::Argument> args, Stream& stream){
        uint8_t* p = nullptr;
        *p = 10;
        return "Done";
    });

    parser.registerCommand("find_ax12", "", [robot](std::vector<CommandParser::Argument> args, Stream& stream) {
        struct Config {
            uint8_t id;
            uint8_t baudrate;
            Config(uint8_t id, uint8_t baudrate) : id(id), baudrate(baudrate) {}
        };
        std::vector<Config> configs = {};
        int baudrates[] = {9600, 19'200, 57'600, 115'200, 200'000, 250'000, 400'000, 500'000, 1'000'000};
        for (int bdrt : baudrates) {
            stream.printf("Testing baudrate %d\r\n", bdrt);
            robot->getAX12Handler()->setBaudrate(bdrt);
            for (int i = 0; i < 254; i++) {
                int data = robot->getAX12Handler()->ping(i);
                if (data != -1)
                    configs.emplace_back(i, bdrt);
                stream.printf("Pinging %d, received %d\r\n", i, data);
                delay(50);
            }
        }
        if (configs.size() != 0) {
            stream.printf("Found %d AX12\r\n", configs.size());
            for (auto config : configs) {
                stream.printf("ID: %d, baudrate: %d\r\n", config.id, config.baudrate);
            }
        }
        return "";
    });

    parser.registerCommand("start_in", "i", [robot](std::vector<CommandParser::Argument> args, Stream& stream) {
        delay(args[0].asInt64());
        robot->setControlDisabled(false);
        return "";
    });


    parser.registerCommand("test_ziegler_nichols", "ioiddd", [robot](std::vector<CommandParser::Argument> args, Stream& stream){
        ZieglerNicholsMethodoTriplePID ziegler(robot, sdMutex, args[0].asInt64(), static_cast<TripleController::SpeedMode>(args[1].asInt64()));
        stream.println("Created ziegler nichols detection class for triple PID");
        double initialValue = args[2].present ? args[2].asDouble() : 2;
        double target = args[3].present ? args[3].asDouble(): (args[0].asInt64() ? 100 : 25);
        double multiplier = args[4].present ? args[4].asDouble() : 1.2;
        ziegler.setInitialValue(initialValue);
        ziegler.setTarget(target);
        ziegler.setMultiplier(multiplier);
        ziegler.start();
        stream.printf("Started ziegler nichols detection with initial value %f and target %f\r\n", initialValue, target);
        waitForMethodoStop(&ziegler, stream);
        ziegler.awaitBeforeDestruction();
        return "Thanks for using ziegler nichols";
    }, PSTR("test_ziegler_nichols <mode> <speed> [initial_P_gain] [target_value] [multiplier] mode=0 : Angle, mode=1: Distance, mode=2 Distance & Angle"));

    parser.registerCommand("test_benchmark", "iodd", [robot](std::vector<CommandParser::Argument> args, Stream& stream) {
        BenchmarkMethodo benchmark(robot, sdMutex, static_cast<BenchmarkMode>(args[0].asInt64()));
        if (args[1]) {
            benchmark.setMultAngle(args[1].asDouble());
        }
        if (args[2]) {
            benchmark.setMultDistance(args[2].asDouble());
        }
        benchmark.start();
        waitForMethodoStop(&benchmark, stream);
        benchmark.awaitBeforeDestruction();
        return PSTR("Thanks for using the benchmark");
    }, PSTR("test_benchmark <mode> [mult_angle] [mult_distance] mode=0 : Angle, mode=1: Distance, mode=2 Distance & Angle"));

    parser.registerCommand(PSTR("test_extremum_seeking"), "ioddddddd", [robot](std::vector<CommandParser::Argument> args, Stream& stream) {
        if (robot->getRobotType() != PAMIRobotType) {
            return PSTR("Your robot is not compatible with the current extremum seeking algorithm");
        }
        ExtremumSeekingMethodo extremum(std::static_pointer_cast<PAMIRobot>(robot), sdMutex, args[0].asInt64());
        stream.println("Testing extremum seeking");

        if(args[1]){
            extremum.setLambda(args[1].asDouble());
            if (args[2]) {
                extremum.setAlphaKP(args[2].asDouble());

                if (args[3]) {
                    extremum.setAlphaKI(args[3].asDouble());

                    if (args[4]) {
                        extremum.setAlphaKD(args[4].asDouble());

                        if (args[5]) {
                            extremum.setGammaKP(args[5].asDouble());

                            if (args[6]) {
                                extremum.setGammaKI(args[6].asDouble());

                                if (args[7]) {
                                    extremum.setGammaKD(args[7].asDouble());
                                }
                            }
                        }
                    }
                }
            }
        }
        stream.println("All things are set starting extremum seeking");
        extremum.start();
        stream.println("Extremum seeking started");
        waitForMethodoStop(&extremum, stream);
        extremum.awaitBeforeDestruction();
        return PSTR("Thanks for using extremum seeking detection");

    },PSTR("test_extremum_seeking <mode> [lambda] [alphaKP] [alphaKI] [alphaKD] [gammaKP] [gammaKI] [gammaKD]"));

    parser.registerCommand(PSTR("test_extremum_seeking_feedforward"), "ioddddddd", [robot](std::vector<CommandParser::Argument> args, Stream& stream) {
        if (robot->getRobotType() != PAMIRobotType) {
            return PSTR("Your robot is not compatible with the current extremum seeking algorithm");
        }
        ExtremumSeekingMethodoFeedForward extremum(std::static_pointer_cast<PAMIRobot>(robot), sdMutex, args[0].asInt64());
        stream.println("Testing extremum seeking");

        if(args[1]){
            extremum.setLambda(args[1].asDouble());
            if (args[2]) {
                extremum.setAlphaKP(args[2].asDouble());

                if (args[3]) {
                    extremum.setAlphaKI(args[3].asDouble());

                    if (args[4]) {
                        extremum.setAlphaKD(args[4].asDouble());

                        if (args[5]) {
                            extremum.setGammaKP(args[5].asDouble());

                            if (args[6]) {
                                extremum.setGammaKI(args[6].asDouble());

                                if (args[7]) {
                                    extremum.setGammaKD(args[7].asDouble());
                                }
                            }
                        }
                    }
                }
            }
        }
        stream.println("All things are set starting extremum seeking");
        extremum.start();
        stream.println("Extremum seeking started");
        waitForMethodoStop(&extremum, stream);
        extremum.awaitBeforeDestruction();
        return PSTR("Thanks for using extremum seeking detection");

    },PSTR("test_extremum_seeking_feedforward <mode> [lambda] [alphaKP] [alphaKI] [alphaKD] [gammaKP] [gammaKI] [gammaKD]"));

    parser.registerCommand("flash_from_sd", "s", [](std::vector<CommandParser::Argument> args, Stream& stream){
        stream.println("Locking mutex");
        sdMutex->lock();
        stream.println("Opening file");
        threads.stop();
        File f= SD.open(args[0].asString().c_str());
        stream.println("Creating buffer");
        char* buffer = (char*) malloc(sizeof(char) * 4096*4);
        stream.println("Creating updater");
        TCPTeensyUpdater updater;
        if(!updater.startFlashMode()){
           stream.printf("Error while starting the flashmode");
        }
        stream.println("Reading bytes ");
        size_t result = f.readBytes(buffer, 4096*4);
        stream.println(result);
        size_t cum_result = result;
        while(result != 0){
            stream.println("Adding data");
            updater.addData(buffer, result);
            stream.printf("Current result %u, cumulative result %u\r\n", result, cum_result);
            if(!updater.parse()){
                return PSTR("Error while parsing");
            }
            result = f.readBytes(buffer, 4096*4);
            cum_result += result;
        }
        stream.printf("Flashing program");
        updater.callDone();

        return PSTR("You shouldn't be seeing this");
    });

    parser.registerCommand("make_square", "ouddddd", [robot](std::vector<CommandParser::Argument> args, Stream& stream) {
        robot->reset_to((0));
        robot->setTranslationalTarget(robot->getTranslationalTarget());
        robot->setRotationalTarget(robot->getRotationalPosition());
        robot->clearTarget();
        robot->controllerClear();
        uint64_t u = args[0].asUInt64Or(1);
        double distance = args[1].asDoubleOr(1000);
        double acc_lin = args[2].asDoubleOr(100);
        double max_speed_lin = args[3].asDoubleOr(200);
        double acc_ang = args[4].asDoubleOr(90);
        double max_speed_ang = args[5].asDoubleOr(180);
        stream.printf("Drawing %u square with the robot\r\n", u);
        for (uint64_t i = 0; i < u; ++i) {
            COMPLETE_POSITION_TARGET(Position(distance,0), RampData(acc_lin,max_speed_lin));
            Position pos(distance,distance);
            COMPLETE_ROTATE_TOWARD_TARGET(pos,RampData(acc_ang,max_speed_ang));
            COMPLETE_POSITION_TARGET(pos, RampData(acc_lin,max_speed_lin));
            pos = Position(0.0,distance);
            COMPLETE_ROTATE_TOWARD_TARGET(pos,RampData(acc_ang,max_speed_ang));
            COMPLETE_POSITION_TARGET(pos, RampData(acc_lin,max_speed_lin));
            pos = Position(0.0, 0.0);
            COMPLETE_ROTATE_TOWARD_TARGET(pos,RampData(acc_ang,max_speed_ang));
            COMPLETE_POSITION_TARGET(pos, RampData(acc_lin,max_speed_lin));
            pos = Position(distance,0.0);
            COMPLETE_ROTATE_TOWARD_TARGET(pos,RampData(acc_ang,max_speed_ang));
        }
        robot->setControlDisabled(false);
        while(robot->getTargetCount() != 0){
            while (stream.available()) {
                char c = stream.read();
                if(c == 'w' || c=='W' || c=='q' || c=='Q' || c == ' '){
                    robot->setControlDisabled(true);
                    robot->lockMotorMutex();
                    robot->getLeftMotor()->setPWM(0);
                    robot->getRightMotor()->setPWM(0);
                    robot->unlockMotorMutex();
                }
            }
        }
        Serial.println("Done");
        return "";
    }, "make_square [distance] [acc_lin] [speed_lin] [acc_ang] [speed_ang]");

    parser.registerCommand("change_distance_to", "u", [robot](std::vector<CommandParser::Argument> args, Stream& stream){
        assert(robot->getRobotType() == PAMIRobotType);
        auto pamirobot = std::static_pointer_cast<PAMIRobot>(robot);
        switch(args[0].asUInt64()){
            case BasicControllerType::PID:
                stream.println("Changing type of controller to PID");
                pamirobot->setControllerDistance(std::make_shared<PID>(robot));
                break;
            case BasicControllerType::PIDSpeedFeedForward:
                stream.println("Changing type of controller to PID Feed forward");
                pamirobot->setControllerDistance(std::make_shared<PIDSpeedFeedForward>(robot));
                break;
            default:
                stream.printf("Unknown type %u\r\n", args[0].asUInt64());
                break;
        }
        return "";
    });
    parser.registerCommand("change_angle_to", "u", [robot](std::vector<CommandParser::Argument> args, Stream& stream){
        assert(robot->getRobotType() == PAMIRobotType);
        auto pamirobot = std::static_pointer_cast<PAMIRobot>(robot);
        switch(args[0].asUInt64()){
            case BasicControllerType::PID:
                stream.println("Changing type of controller to PID");
                pamirobot->setControllerAngle(std::make_shared<PID>(robot));
                break;
            case BasicControllerType::PIDSpeedFeedForward:
                stream.println("Changing type of controller to PID Feed forward");
                pamirobot->setControllerAngle(std::make_shared<PIDSpeedFeedForward>(robot));
                break;
            default:
                stream.printf("Unknown type %u\r\n", args[0].asUInt64());
                break;
        }
        return "";
    });
    parser.registerCommand("change_distance_angle_to", "u", [robot](std::vector<CommandParser::Argument> args, Stream& stream){
        assert(robot->getRobotType() == PAMIRobotType);
        auto pamirobot = std::static_pointer_cast<PAMIRobot>(robot);
        switch(args[0].asUInt64()){
            case BasicControllerType::PID:
                stream.println("Changing type of controller to PID");
                pamirobot->setControllerDistanceAngle(std::make_shared<PID>(robot));
                break;
            case BasicControllerType::PIDSpeedFeedForward:
                stream.println("Changing type of controller to PID Feed forward");
                pamirobot->setControllerDistanceAngle(std::make_shared<PIDSpeedFeedForward>(robot));
                break;
            default:
                stream.printf("Unknown type %u\r\n", args[0].asUInt64());
                break;
        }
        return "";
    });

    AX12_CONTROL_TABLE
}