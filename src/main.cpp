//#define _TEENSY41_ASYNC_TCP_LOGLEVEL_     5


#include "TeensyThreads.h"
#include <Arduino.h>
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

#include "CommandParser.h"
#include "utils/ThreadPool.h"
#include "utils/TaskScheduler.h"
#include "ramp/CalculatedQuadramp.h"
#include "target/AngleTarget.h"
#include "target/PositionTarget.h"
#include "utils/SDFlashMem.h"

//#define ENABLE_WEB_SERVER_OTA
std::shared_ptr<std::thread> robot_update;
std::shared_ptr<std::thread> scheduler_update;
std::shared_ptr<std::thread> command_line_update;
std::shared_ptr<ThreadPool> threadPool;
std::shared_ptr<TaskScheduler> scheduler;


std::shared_ptr<PAMIRobot> robot;
std::shared_ptr<CommandLineHandler> cmd_line_handler;
std::shared_ptr<BufferFilePrint> bufferPrinter;
std::vector<std::shared_ptr<BufferFilePrint>> bufferPrinters;
std::map<uint16_t, std::shared_ptr<CustomAsyncClient>> customAsyncClientMap;

File f;
std::shared_ptr<AsyncServer> server;
using namespace std::chrono;
bool flashing_process = false;

CommandParser parser;
PacketHandler packetHandler;

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

[[noreturn]] void handle_command_line() {
    while (true) {
        cmd_line_handler->handle_commandline();
        Threads::yield();
        threads.delay_us(100);
    }
}

[[noreturn]] void handle_scheduler() {
    while (true) {
        scheduler->update();
        threads.delay_us(100);
    }
}

void setup() {
    threadPool = std::make_shared<ThreadPool>(6);
    scheduler = std::make_shared<TaskScheduler>(threadPool);
    initSDCard();
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

    /* check for CrashReport stored from previous run */
    if (CrashReport) {
        /* print info (hope Serial Monitor windows is open) */
        Serial.print(CrashReport);
    }
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

    Serial.println("Initialising command line updater");
    command_line_update = std::make_shared<std::thread>(handle_command_line);

    Serial.println("Initialising Scheduler updater");
    scheduler_update = std::make_shared<std::thread>(handle_scheduler);
    scheduler_update->detach();
    Serial.println("Initialising robot updater");
    robot_update = std::make_shared<std::thread>(handle_robot_update);
    robot_update->detach();

    scheduler->addTask(milliseconds(100), []() {
        for (auto& buffered : bufferPrinters) {
            buffered->flush();
        }
    }, milliseconds(100));

    robot->addTarget(std::make_shared<PositionTarget<CalculatedQuadramp>>(robot, Position(1000,0), RampData(100, 200)));
    robot->addTarget(std::make_shared<AngleTarget<CalculatedQuadramp>>(robot, Angle::fromDegrees(180), RampData(45, 90)));
    robot->addTarget(std::make_shared<PositionTarget<CalculatedQuadramp>>(robot, Position(0,0), RampData(100, 200)));
    /*
    std::shared_ptr<BaseTarget> base_target = std::make_shared<AngleTarget<CalculatedQuadramp>>(robot, 90_deg, RampData(45,90));
    base_target->addEndCallback([]() {
        scheduler->addTask(std::chrono::seconds(2), []() {
            Serial.println("End of base target");
            robot->addTarget(std::make_shared<AngleTarget<CalculatedQuadramp>>(robot, 0_deg, RampData(45,90)));
        });
    });
    robot->addTarget(base_target);
    */




}
void loop() {

}