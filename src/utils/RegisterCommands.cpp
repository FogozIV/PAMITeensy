//
// Created by fogoz on 13/05/2025.
//
#include "utils/RegisterCommands.h"

#include "utils/InteractContext.h"

FLASHMEM void registerCommands(CommandParser &parser, std::shared_ptr<BaseRobot> robot) {
    parser.registerCommand("hello", "", [](std::vector<CommandParser::Argument> arg, Stream& stream) {
        return "hello world !";
    }, PSTR("a test command that says hello world"));

    parser.registerCommand("help", "", [&parser](std::vector<CommandParser::Argument> arg, Stream& stream) {
        for (CommandParser::Command cmd: parser.command_definitions()) {
            stream.println((cmd.name + ": " + (strlen(cmd.description) == 0 ? "No description found" : cmd.description)).c_str());
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

    parser.registerCommand("flash", "", [](std::vector<CommandParser::Argument> args, Stream& stream) {
        stream.print("Beginning the flash : ");
        if (flashing_process) {
            return PSTR("unable to flash another process is already doing it");
        }
        flashing_process = true;
        uint32_t buffer_addr, buffer_size;
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

    parser.registerCommand("get_target_curvilinear", "", [robot](std::vector<CommandParser::Argument> args, Stream& stream) {
        stream.printf("Curvilinear target : %f\r\n", robot->getTranslationalTarget());
        return "";
    });

    parser.registerCommand("set_target_curvilinear", "d", [robot](std::vector<CommandParser::Argument> args, Stream& stream){
        robot->setTranslationalTarget(args[0].asDouble());
        return "";
    });

    parser.registerCommand("get_position_curvilinear", "", [robot](std::vector<CommandParser::Argument> args, Stream& stream) {
        stream.printf("Curvilinear position : %f\r\n", robot->getTranslationalPosition());
        return "";
    });

    parser.registerCommand("get_target_angle", "", [robot](std::vector<CommandParser::Argument> args, Stream& stream) {
        stream.printf("Angle target : %f\r\n", robot->getRotationalTarget());
        return "";
    });

    parser.registerCommand("get_position_angle", "", [robot](std::vector<CommandParser::Argument> args, Stream& stream) {
        stream.printf("Angle position : %f\r\n", robot->getRotationalPosition());
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

    parser.registerCommand("test_pwm", "", [robot](std::vector<CommandParser::Argument> args, Stream& stream){
        robot->setControlDisabled(true);
        for(int i = 94; i < 4095; i+=200){
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

    AX12_CONTROL_TABLE
}