

#include <Arduino.h>

#include "robot/PAMIRobot.h"
#include "CommandParser.h"
#include "TeensyThreads.h"
#include "utils/RegisterCommands.h"
#include "utils/BufferFilePrint.h"
#include "ramp/DynamicQuadramp.h"
#include "utils/HeaderPrint.h"

std::shared_ptr<PAMIRobot> robot;
std::shared_ptr<std::thread> usb_command_line;
std::shared_ptr<std::thread> sd_writer;
std::shared_ptr<std::thread> robot_update;
std::vector<std::shared_ptr<BufferFilePrint>> bufferPrinters;


using namespace std::chrono;

[[noreturn]] void handle_sd_card(){
    auto data_point = steady_clock::now();
    while(true){
        if(steady_clock::now() - data_point < 100ms){
            Threads::yield();
            continue;
        }
        data_point = steady_clock::now();
        for(auto& buffered : bufferPrinters){
            buffered->flush();
        }
    }
}

[[noreturn]] void handle_robot_update(){
    auto data_point = steady_clock::now();
    while(true){
        if(steady_clock::now() - data_point < 5ms){
            Threads::yield();
            continue;
        }
        data_point = steady_clock::now();
        robot->compute();
    }
}

CommandParser parser;

void setup() {
    Serial.begin(1000000);
    /* check for CrashReport stored from previous run */
    if (CrashReport) {
        /* print info (hope Serial Monitor windows is open) */
        Serial.print(CrashReport);
    }

    printHeader();
    Serial.printf("Hello world ! Welcome to the teensy, it was compiled the %s at %s \r\n", __DATE__, __TIME__);
    Serial.println("FogozIV was here");
    Serial.println("OTA Working without confirmation and hello world");
    robot = std::make_shared<PAMIRobot>();
    robot->init(robot);
    registerCommands(parser, robot, Serial);

    threads.setDefaultStackSize(10000);
    threads.setDefaultTimeSlice(10);
    threads.setSliceMicros(10);

    usb_command_line = std::make_shared<std::thread>(handle_commandline, &parser);
    usb_command_line->detach();

    sd_writer = std::make_shared<std::thread>(handle_sd_card);
    sd_writer->detach();

    robot_update = std::make_shared<std::thread>(handle_robot_update);
    robot_update->detach();

    robot->setControlDisabled(true);

}

void loop() {
}