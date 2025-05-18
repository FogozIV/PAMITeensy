//
// Created by fogoz on 13/05/2025.
//
#include "utils/RegisterCommands.h"

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
        SCB_AIRCR = 0x05FA0004;
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
        stream.println("Use arrow key to move (if movement is not in the right direction please use the calib_motors command)");
        stream.println("Use space to stop robot movement");
        stream.println("Use q, Q, w, W to exit the interact");
        bool controlDisabled = robot->isControlDisabled();
        robot->setControlDisabled(true);
        StateMachine stateMachine;
        double leftPWM = 0;
        double rightPWM = 0;
        double maxPWMLeft = robot->getLeftMotor()->getMaxPWM();
        double maxPWMRight = robot->getRightMotor()->getMaxPWM();
        stateMachine.set(UP_LAST_CHAR, [&]{
            leftPWM += maxPWMLeft * 0.1;
            rightPWM += maxPWMRight * 0.1;
        });
        stateMachine.set(DOWN_LAST_CHAR, [&]{
            leftPWM -= maxPWMLeft * 0.1;
            rightPWM -= maxPWMRight * 0.1;
        });
        stateMachine.set(LEFT_LAST_CHAR, [&]{
            leftPWM -= maxPWMLeft * 0.1;
            rightPWM += maxPWMRight * 0.1;
        });
        stateMachine.set(RIGHT_LAST_CHAR, [&]{
            leftPWM += maxPWMLeft * 0.1;
            rightPWM -= maxPWMRight * 0.1;
        });
        while(true){
            while(stream.available()){
                char c = stream.read();
                if(c == 27){
                    stateMachine.begin();
                    continue;
                }
                if(stateMachine.isStarted()){
                    stateMachine.append(c);
                    if(stateMachine.isStarted()){
                        continue;
                    }
                }
                if(c == 32){
                    leftPWM = 0;
                    rightPWM = 0;
                }

                if(c == 'w' || c=='W' || c=='q' || c=='Q'){
                    robot->getLeftMotor()->setPWM(0);
                    robot->getRightMotor()->setPWM(0);
                    robot->setControlDisabled(controlDisabled);
                    return "Thanks for using interact";
                }
                leftPWM = constrain(leftPWM, -maxPWMLeft, maxPWMLeft);
                rightPWM = constrain(rightPWM, -maxPWMRight, maxPWMRight);
                stream.printf("Left pwm : %f, Right pwm: %f \r\n", leftPWM, rightPWM);
                robot->getLeftMotor()->setPWM(leftPWM);
                robot->getRightMotor()->setPWM(rightPWM);
            }

            Threads::yield();
        }

    });

    parser.registerCommand("position", "", [robot](std::vector<CommandParser::Argument> args, Stream& stream){
        stream.print("Robot current pos: ");
        robot->getCurrentPosition().printTo(stream);
        return " ";
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

    parser.registerCommand("stop", "", [robot](std::vector<CommandParser::Argument> args, Stream& stream) {
        robot->setControlDisabled(true);
        robot->getLeftMotor()->setPWM(0);
        robot->getRightMotor()->setPWM(0);
        return "";
    });

    parser.registerCommand("save", "", [robot](std::vector<CommandParser::Argument> args, Stream& stream) {
        robot->save();
        return PSTR("The configuration has been saved");
    });
    parser.registerCommand("rotateToward", "ddd", [robot](std::vector<CommandParser::Argument> args, Stream& stream) {
        robot->addTarget(std::make_shared<AngleTarget<CalculatedQuadramp>>(robot, Angle::fromDegrees(args[0].asDouble()), RampData(args[1].asDouble(), args[2].asDouble())));
        return "";
    });

    parser.registerCommand("moveToward", "dddd", [robot](std::vector<CommandParser::Argument> args, Stream& stream) {
        robot->addTarget(std::make_shared<PositionTarget<CalculatedQuadramp>>(robot, Position(args[0].asDouble(), args[1].asDouble()), RampData(args[2].asDouble(), args[3].asDouble())));
        return "";
    });

    AX12_CONTROL_TABLE
}