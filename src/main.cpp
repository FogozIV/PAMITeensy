//#define _TEENSY41_ASYNC_TCP_LOGLEVEL_     5
#include <Arduino.h>
#include <map>

#ifndef DISABLE_COMMAND_LINE
#include "CommandParser.h"
#endif

#include "TeensyThreads.h"
#ifndef DISABLE_COMMAND_LINE
#include "utils/RegisterCommands.h"
#endif
#ifndef DISABLE_NETWORKING
#include "network/CustomAsyncClient.h"
#include "QNEthernet.h"
#include "utils/NetworkUtils.h"
#include "Teensy41_AsyncTCP.h"
#endif
#include "robot/PAMIRobot.h"
#include "utils/BufferFilePrint.h"
#include "utils/HeaderPrint.h"
#include "utils/CRC.h"
//#define ENABLE_WEB_SERVER_OTA

std::shared_ptr<PAMIRobot> robot;
std::shared_ptr<std::thread> usb_command_line;
std::shared_ptr<std::thread> sd_writer;
std::shared_ptr<std::thread> robot_update;
std::vector<std::shared_ptr<BufferFilePrint>> bufferPrinters;
#ifndef DISABLE_NETWORKING
std::map<uint16_t, std::shared_ptr<CustomAsyncClient>> customAsyncClientMap;
#endif
#ifdef ENABLE_WEB_SERVER_OTA
#include "AsyncWebServer_Teensy41.hpp"
#include "EthernetUpload.h"
std::shared_ptr<AsyncWebServer> webServer;
std::shared_ptr<TeensyOtaUpdater> updater;
#endif
#ifndef DISABLE_NETWORKING
std::shared_ptr<AsyncServer> server;
#endif
using namespace std::chrono;
bool flashing_process = false;

#ifndef DISABLE_COMMAND_LINE
CommandParser parser;
CommandLineHandler cmd_line_handler(parser, Serial);
#endif
#ifndef DISABLE_NETWORKING
PacketHandler packetHandler;
#endif
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

#ifndef DISABLE_COMMAND_LINE
[[noreturn]] void handle_usb_command() {
    while (true) {
        cmd_line_handler.handle_commandline();
    }
}
#endif

void setup() {
    /*
     * Threads settings to avoid stack overflow and threads definition to handle various tasks
     */
    threads.setDefaultStackSize(10000);
    threads.setDefaultTimeSlice(10);
    threads.setSliceMicros(10);
    Serial.begin(1000000);
    delay(1000);
    /* check for CrashReport stored from previous run */
    if (CrashReport) {
        /* print info (hope Serial Monitor windows is open) */
        Serial.print(CrashReport);
    }
    printHeader();
#ifndef DISABLE_NETWORKING
    CustomEthernetStatus status = setupEthernet();
    if (status == CustomEthernetStatus::OK) {
        Serial.println("Ethernet initialized");
        server = std::make_shared<AsyncServer>(80);
        server->onClient([](void* _, AsyncClient * client) {
            Serial.printf("New client connected %d\r\n", client->getConnectionId());
            if (customAsyncClientMap.count(client->getConnectionId()) == 0) {
                auto customClient = std::make_shared<CustomAsyncClient>(client);
                customAsyncClientMap.emplace(client->getConnectionId(), customClient);
                customClient->sendPing(random());
            }
        }, nullptr);
        server->begin();
        #ifdef ENABLE_WEB_SERVER_OTA
        auto [ws, up] = setupWebServer();
        webServer = ws;
        updater = up;
        webServer->begin();
        #endif
    }
#endif
    Serial.println(algoCRC_8.computeCRC("123456789"));
    Serial.printf("Hello world ! Welcome to the teensy, it was compiled the %s at %s \r\n", __DATE__, __TIME__);
    /*
     * Create the robot and initialize it, this will also create the motors and the servos
     */
    robot = std::make_shared<PAMIRobot>();
    robot->init();
    /*
     * Register the commands that will be available in the command line
     */
#ifndef DISABLE_COMMAND_LINE
    registerCommands(parser, robot);
#endif
    //robot->registerCommands(parser);





#ifndef DISABLE_COMMAND_LINE
    usb_command_line = std::make_shared<std::thread>(handle_usb_command);
    usb_command_line->detach();
#endif
    sd_writer = std::make_shared<std::thread>(handle_sd_card);
    sd_writer->detach();

    robot_update = std::make_shared<std::thread>(handle_robot_update);
    robot_update->detach();

    /*
     * Disable the control of the robot, we will use the command line to control the robot
     */
    robot->setControlDisabled(true);

}
void loop() {

}