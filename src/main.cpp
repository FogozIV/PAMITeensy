

#include <Arduino.h>
#include "robot/PAMIRobot.h"
#include "CommandParser.h"
#include "TeensyThreads.h"
#include "utils/RegisterCommands.h"
#include "utils/BufferFilePrint.h"
#include "utils/AX12.h"

std::shared_ptr<PAMIRobot> robot;
std::shared_ptr<std::thread> usb_command_line;
std::shared_ptr<std::thread> sd_writer;
std::shared_ptr<std::thread> robot_update;
std::vector<std::shared_ptr<BufferFilePrint>> bufferPrinters;


using namespace std::chrono;

[[noreturn]] void handle_sd_card(){
    auto data_point = std::chrono::steady_clock::now();
    while(true){
        if(std::chrono::steady_clock::now() - data_point < 100ms){
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
/*
 //working
void test() {
    Serial2.begin(115200, SERIAL_8N1 | SERIAL_HALF_DUPLEX);
    Serial2.write(0xFF);
    Serial2.write(0xFF);
    Serial2.write(0x0A);
    Serial2.write(0x02);
    Serial2.write(0x01);
    Serial2.write(~((0x0A+0x02+0x01)&0XFF));
    Serial2.flush();
    while (!(Serial2.available() >2)) {
        Threads::yield();
    }
    Serial.println("Response");
    while (Serial2.available()) {
        Serial.printf("%02x ", Serial2.read());
    }

    Serial.println();
}
*/

CommandParser parser;

void setup() {
    Serial.begin(1000000);
    /* check for CrashReport stored from previous run */
    if (CrashReport) {
        /* print info (hope Serial Monitor windows is open) */
        Serial.print(CrashReport);
    }

    Serial.printf("Hello world ! Welcome to the teensy it was compiled the %s at %s \r\n", __DATE__, __TIME__);

    robot = std::make_shared<PAMIRobot>();
    robot->init(robot);
    registerCommands(parser, robot);


    threads.setDefaultStackSize(10000);
    threads.setDefaultTimeSlice(10);
    threads.setSliceMicros(10);

    usb_command_line = std::make_shared<std::thread>(handle_commandline, &parser);
    usb_command_line->detach();

    sd_writer = std::make_shared<std::thread>(handle_sd_card);
    sd_writer->detach();

    robot_update = std::make_shared<std::thread>(handle_robot_update);
    robot_update->detach();

    robot->save();

    auto ax_id10 = robot->getAX12Handler()->get(10);
    ax_id10.writeTORQUE_ENABLE(1);
    ax_id10.writeGOAL_POSITION(0);
    ax_id10.writeCCW_ANGLE_LIMIT(1023);
    ax_id10.writeCW_ANGLE_LIMIT(0);
    ax_id10.writeLED(0);

}

void loop() {
}