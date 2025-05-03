#define _TEENSY41_ASYNC_TCP_LOGLEVEL_     5
#include <Arduino.h>

#include "robot/PAMIRobot.h"
#include "CommandParser.h"
#include "TeensyThreads.h"
#include "utils/RegisterCommands.h"
#include "utils/BufferFilePrint.h"
#include "ramp/DynamicQuadramp.h"
#include "ramp/BasicQuadramp.h"
#include "utils/HeaderPrint.h"
#include "QNEthernet.h"
#include "utils/NetworkUtils.h"
#include "Teensy41_AsyncTCP.h"
#include "utils/CRC.h"
#include "utils/TCPStateMachine.h"
#include <unordered_map>

//#define ENABLE_WEB_SERVER_OTA

std::shared_ptr<PAMIRobot> robot;
std::shared_ptr<std::thread> usb_command_line;
std::shared_ptr<std::thread> sd_writer;
std::shared_ptr<std::thread> robot_update;
std::vector<std::shared_ptr<BufferFilePrint>> bufferPrinters;

#ifdef ENABLE_WEB_SERVER_OTA
#include "AsyncWebServer_Teensy41.hpp"
#include "EthernetUpload.h"
std::shared_ptr<AsyncWebServer> webServer;
std::shared_ptr<TeensyOtaUpdater> updater;
#endif

std::shared_ptr<AsyncServer> server;

using namespace std::chrono;

CommandParser parser;
CommandLineHandler cmd_line_handler(parser, Serial);

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

[[noreturn]] void handle_usb_command() {
    while (true) {
        cmd_line_handler.handle_commandline();
    }
}

std::unordered_map<uint16_t, std::shared_ptr<TCPStateMachine>> tcp_state_machines;

void setup() {
    /*
     * Threads settings to avoid stack overflow and threads definition to handle various tasks
     */
    threads.setDefaultStackSize(10000);
    threads.setDefaultTimeSlice(10);
    threads.setSliceMicros(10);
    delay(5000);
    Serial.begin(1000000);
    /* check for CrashReport stored from previous run */
    if (CrashReport) {
        /* print info (hope Serial Monitor windows is open) */
        Serial.print(CrashReport);
    }
    printHeader();
    CustomEthernetStatus status = setupEthernet();
    if (status == CustomEthernetStatus::OK) {
        Serial.println("Ethernet initialized");
        server = std::make_shared<AsyncServer>(80);
        TCPStateMachine::registerListeners();
        server->onClient([](void* _, AsyncClient * client) {
            tcp_state_machines.emplace(client->getConnectionId(), std::make_shared<TCPStateMachine>());
            client->onDisconnect([](void* _, AsyncClient * client) {
                tcp_state_machines.erase(client->getConnectionId());
            });
            client->onData([](void* _, AsyncClient * client, void * data, size_t len) {
                tcp_state_machines.at(client->getConnectionId())->handleData(client, data, len);

            });

        }, nullptr);
        server->begin();
        #ifdef ENABLE_WEB_SERVER_OTA
        auto [ws, up] = setupWebServer();
        webServer = ws;
        updater = up;
        webServer->begin();
        #endif
    }
    Serial.println(algoCRC_8.computeCRC("123456789"));
    Serial.printf("Hello world ! Welcome to the teensy, it was compiled the %s at %s \r\n", __DATE__, __TIME__);
    /*
     * Create the robot and initialize it, this will also create the motors and the servos
     */
    robot = std::make_shared<PAMIRobot>();
    robot->init(robot);
    /*
     * Register the commands that will be available in the command line
     */
    registerCommands(parser, robot);





    usb_command_line = std::make_shared<std::thread>(handle_usb_command);
    usb_command_line->detach();

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