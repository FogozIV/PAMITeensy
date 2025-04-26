//
// Created by fogoz on 25/04/2025.
//

#ifndef REGISTERCOMMANDS_H
#define REGISTERCOMMANDS_H

#include "CommandParser.h"

inline void registerCommands(CommandParser& parser, std::shared_ptr<BaseRobot> robot) {


    parser.registerCommand("hello", "", [](std::vector<CommandParser::Argument> arg) {
        return "hello world !";
    }, "a test command that says hello world");

    parser.registerCommand("help", "", [&parser](std::vector<CommandParser::Argument> arg) {
        std::string result;
        for (CommandParser::Command cmd : parser.command_definitions()) {
            result += cmd.name + ": " + (cmd.description.empty() ? "No description found" : cmd.description) + "\n";
        }
        result.pop_back();
        return result;
    }, "a command that displays all the available commands");

    parser.registerCommand("encoder_calib_rotation_turn", "d", [robot](std::vector<CommandParser::Argument> arg) {
        robot->endCalibrationAngleTurnEncoder(arg[0].asDouble());
        return "success";
    }, "a command that allows you to type in the number of turn the robot did and calibrate it automatically don't forget to call encoder_calib_init before calling this function");

    parser.registerCommand("encoder_calib_init", "", [robot](std::vector<CommandParser::Argument> arg) {
        robot->beginCalibrationEncoder();
        return "Started the encoder calibration please type encoder_calib_rotation_turn or encoder_calib_straight after moving the robot";
    }, "a command that allows you to start the calibration of the encoder");

    parser.registerCommand("encoder_calib_straight", "d", [robot](std::vector<CommandParser::Argument> arg) {
        robot->endCalibrationStraightEncoder(arg[0].asDouble());
        return "success";
    }, "a command that allows you to type in the distance the robot did and calibrate it automatically don't forget to call encoder_calib_init before calling this function");

    parser.registerCommand("calib_motors", "", [robot](std::vector<CommandParser::Argument> args){
        robot->calibrateMotors();
        return "success";
    }, "a command that allows you to correct the motor wiring internally");
}
#endif //REGISTERCOMMANDS_H
