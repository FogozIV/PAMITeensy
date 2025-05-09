//#define _TEENSY41_ASYNC_TCP_LOGLEVEL_     5


#include "TeensyThreads.h"
#include <Arduino.h>

#include "CommandParser.h"
#include "utils/RegisterCommands.h"
#include "network/CustomAsyncClient.h"
#include "QNEthernet.h"
#include "utils/NetworkUtils.h"
#include "Teensy41_AsyncTCP.h"
#include "robot/PAMIRobot.h"
#include <map>
#include <chrono>
#include "utils/HeaderPrint.h"
#include "utils/BufferFilePrint.h"
#include <memory>

#include "ramp/CalculatedQuadramp.h"
#include "target/AngleTarget.h"
#include "target/PositionTarget.h"
//#include "utils/TaskScheduler.h"

//#define ENABLE_WEB_SERVER_OTA
std::shared_ptr<PAMIRobot> robot;
std::shared_ptr<std::thread> usb_command_line;
std::shared_ptr<std::thread> sd_writer;
std::shared_ptr<std::thread> robot_update;
//std::shared_ptr<std::thread> scheduler;
std::vector<std::shared_ptr<BufferFilePrint>> bufferPrinters;
std::map<uint16_t, std::shared_ptr<CustomAsyncClient>> customAsyncClientMap;
File f;
#ifdef ENABLE_WEB_SERVER_OTA
#include "AsyncWebServer_Teensy41.hpp"
#include "EthernetUpload.h"
std::shared_ptr<AsyncWebServer> webServer;
std::shared_ptr<TeensyOtaUpdater> updater;
#endif
std::shared_ptr<AsyncServer> server;
using namespace std::chrono;
bool flashing_process = false;

CommandParser parser;
std::shared_ptr<CommandLineHandler> cmd_line_handler;
std::shared_ptr<BufferFilePrint> bufferPrinter;
PacketHandler packetHandler;
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
    auto initial = std::chrono::steady_clock::now();
    uint64_t n = 0;
    while(true){
        if(steady_clock::now() - initial < n*5ms){
            Threads::yield();
            continue;
        }
        n++;
        robot->compute();
    }
}

[[noreturn]] void handle_usb_command() {
    while (true) {
        cmd_line_handler->handle_commandline();
        Threads::yield();
    }
}
/*
[[noreturn]] void handle_scheduler() {
    while (true) {
        taskScheduler.update();
        threads.delay_us(100);
    }
}*/

void setup() {
    /* check for CrashReport stored from previous run */
    if (CrashReport) {
        /* print info (hope Serial Monitor windows is open) */
        Serial.print(CrashReport);
    }
    SD.begin(BUILTIN_SDCARD);
    f = SD.open((String(rtc_get()) + ".txt").c_str(), FILE_WRITE_BEGIN);
    bufferPrinter = std::make_shared<BufferFilePrint>(f, 8192);
    bufferPrinters.push_back(bufferPrinter);
    /*
     * Threads settings to avoid stack overflow and threads definition to handle various tasks
     */
    threads.setDefaultStackSize(10000);
    threads.setDefaultTimeSlice(10);
    threads.setSliceMicros(10);
    Serial.begin(1000000);
    delay(1000);
    cmd_line_handler = std::make_shared<CommandLineHandler>(parser, Serial);
    printHeader();
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

    Serial.printf("Hello world ! Welcome to the teensy, it was compiled the %s at %s \r\n", __DATE__, __TIME__);
    /*
     * Create the robot and initialize it, this will also create the motors and the servos
     */
    robot = std::make_shared<PAMIRobot>();
    robot->init();
    /*
     * Register the commands that will be available in the command line
     */
    registerCommands(parser, robot);
    robot->registerCommands(parser);

    /*
     * Disable the control of the robot, we will use the command line to control the robot
     */
    robot->setControlDisabled(true);

    Serial.println("Initialising serial command");
    usb_command_line = std::make_shared<std::thread>(handle_usb_command);
    usb_command_line->detach();

    Serial.println("Initialising sd card buffers");
    sd_writer = std::make_shared<std::thread>(handle_sd_card);
    sd_writer->detach();


    robot_update = std::make_shared<std::thread>(handle_robot_update);
    robot_update->detach();
/*
    scheduler = std::make_shared<std::thread>(handle_scheduler);
    scheduler->detach();
*/

    std::shared_ptr<BaseTarget> base_target = std::make_shared<AngleTarget<CalculatedQuadramp>>(robot, 90_deg, RampData(45,90));
    base_target->addEndCallback([]() {
        /*taskScheduler.addTask(std::chrono::seconds(2), []() {
            robot->addTarget(std::make_shared<AngleTarget<CalculatedQuadramp>>(robot, 0_deg, RampData(45,90)));
        });*/
    });
    robot->addTarget(base_target);





}
void loop() {

}