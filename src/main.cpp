

#include <Arduino.h>
#include "robot/PAMIRobot.h"
#include "CommandParser.h"
#include "TeensyThreads.h"
#include "utils/RegisterCommands.h"
std::shared_ptr<PAMIRobot> robot;
std::shared_ptr<std::thread> usb_command_line;

CommandParser parser;

void setup() {
    Serial.begin(1000000);
    /* check for CrashReport stored from previous run */
    if (CrashReport) {
        /* print info (hope Serial Monitor windows is open) */
        Serial.print(CrashReport);
    }
    robot = std::make_shared<PAMIRobot>();
    robot->init(robot);
    registerCommands(parser, robot);


    threads.setDefaultStackSize(10000);
    threads.setDefaultTimeSlice(10);
    threads.setSliceMicros(10);

    usb_command_line = std::make_shared<std::thread>(handle_commandline, &parser);
    usb_command_line->detach();
    robot->save();

}

void loop() {

}