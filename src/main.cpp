//#define _TEENSY41_ASYNC_TCP_LOGLEVEL_     5
#include "utils/config.h"
#include "TeensyThreads.h"
#include <Arduino.h>
#ifdef DEBUG_MODE_CUSTOM
#include "TeensyDebug.h"
#endif
#include "utils/StreamSplitter.h"
#include "CommandParser.h"
#include "utils/RegisterCommands.h"
#include "network/CustomAsyncClient.h"
#include "QNEthernet.h"
#include "utils/NetworkUtils.h"
#include "Teensy41_AsyncTCP.h"
#include "robot/PAMIRobot.h"
#include <chrono>
#include "utils/HeaderPrint.h"
#include "utils/BufferFilePrint.h"
#include <memory>

#include "utils/ThreadPool.h"
#include "utils/TaskScheduler.h"
#include "ramp/CalculatedQuadramp.h"
#include "target/AngleTarget.h"
#include "target/RotateTowardTarget.h"
#include "target/PositionTarget.h"
#include "target/RelativePositionTarget.h"
#include "curves/ClothoidCurve.h"

#include "curves/CurveList.h"
#include <CrashReport.h>

#include "target/CurveTarget.h"
#include "target/FunctionTarget.h"


std::shared_ptr<std::mutex> sdMutex;
std::shared_ptr<std::mutex> motorMutex;
//#define ENABLE_WEB_SERVER_OTA
std::shared_ptr<std::thread> robot_update;
std::shared_ptr<std::thread> scheduler_update;
std::shared_ptr<std::thread> command_line_update;
std::shared_ptr<ThreadPool> threadPool;
std::shared_ptr<TaskScheduler> scheduler;
bool pause_thread_info = false;

extern "C" char *__sbrk(int incr);
int FreeRam() {
    char stackDummy;
    void* heapEnd = malloc(4);  // allocate 4 bytes to get heap end
    free(heapEnd);              // immediately free it
    return &stackDummy - (char*)heapEnd;
}
std::shared_ptr<PAMIRobot> robot;
std::shared_ptr<BaseRobot> base_robot; ///Necessary because used as a global variable to generate some targets and make it easier (should be the same instance as robot)
std::shared_ptr<CommandLineHandler> cmd_line_handler;
std::shared_ptr<CommandLineHandler> xbeeCommandParserHandler;
std::shared_ptr<BufferFilePrint> bufferPrinter;
std::vector<std::shared_ptr<BufferFilePrint>> bufferPrinters;
std::vector<std::shared_ptr<CustomAsyncClient>> customAsyncClientMap;

File f;
std::shared_ptr<AsyncServer> server;
using namespace std::chrono;
bool flashing_process = false;

CommandParser parser;
CommandParser xbeeCommandParser;
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
#ifndef DEBUG_MODE_CUSTOM
        xbeeCommandParserHandler->handle_commandline();
#endif
        if(Serial8.available())
            streamSplitter.println(Serial8.readString());
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

void FLASHMEM setupPROGMEM() {
    auto status = setupEthernet();
    if (status == CustomEthernetStatus::OK) {
        streamSplitter.println("LOG= Ethernet initialized");
        server = std::make_shared<AsyncServer>(80);
        server->onClient([](void* _, AsyncClient * client) {
            streamSplitter.printf("LOG= New client connected %d\r\n", client->getConnectionId());
            if (customAsyncClientMap.size() < client->getConnectionId()) {
                auto customClient = std::make_shared<CustomAsyncClient>(client);
                customAsyncClientMap.emplace_back(customClient);
                customClient->sendPing(random());
            }
        }, nullptr);
        server->begin();
    }

    streamSplitter.printf(F("LOG= Hello world ! Welcome to the teensy coded by FogozIV, it was compiled the %s at %s \r\n"), __DATE__, __TIME__);
    /*
     * Create the robot and initialize it, this will also create the motors and the servos
     */
    streamSplitter.println(F("LOG= Creating robot"));
    robot = std::make_shared<PAMIRobot>(motorMutex);
    base_robot = robot;
    robot->init();
    /*
     * Register the commands that will be available in the command line
     */
    streamSplitter.println(F("LOG= Registering commands"));
    registerCommands(xbeeCommandParser, robot);
    registerCommands(parser, robot);
    robot->registerCommands(xbeeCommandParser);
    robot->registerCommands(parser);

    /*
     * Disable the control of the robot, we will use the command line to control the robot
     */
    robot->setControlDisabled(true);
    streamSplitter.println(F("LOG= Initialising command line updater"));
    command_line_update = std::make_shared<std::thread>(handle_command_line);
    command_line_update->detach();
    streamSplitter.println(F("Initialising Scheduler updater"));
    scheduler_update = std::make_shared<std::thread>(handle_scheduler);
    scheduler_update->detach();
    streamSplitter.println(F("Initialising robot updater"));
    robot_update = std::make_shared<std::thread>(handle_robot_update);
    robot_update->detach();

    scheduler->addTask(milliseconds(100), []() {
        for (auto& buffered : bufferPrinters) {
            buffered->flush();
        }
    }, milliseconds(100));

    scheduler->addTask(seconds(5), []() {
        if(!pause_thread_info){
            streamSplitter.println(threads.threadsInfo());
        }
        }, seconds(20));
    streamSplitter.println(F("Done initialising"));
}

void setup() {
    for(int i = 0; i < 42; i++){
        pinMode(i, INPUT);
    }
    motorMutex = std::make_shared<std::mutex>();

    /*
     * Threads settings to avoid stack overflow and threads definition to handle various tasks
     */
    threads.setDefaultStackSize(4000);
    threads.setDefaultTimeSlice(10);
    threads.setSliceMicros(10);
    Serial.begin(1000000);
    Serial7.begin(115200, SERIAL_8N1);
#ifdef DEBUG_MODE_CUSTOM
    delay(1000);
    debug.begin(Serial7);
#endif
    Serial8.begin(38400);


    threadPool = std::make_shared<ThreadPool>(3);
    scheduler = std::make_shared<TaskScheduler>(threadPool);
    sdMutex = std::make_shared<std::mutex>();
    sdMutex->lock();
    SD.begin(BUILTIN_SDCARD);
    f = SD.open((String(rtc_get()) + ".txt").c_str(), FILE_WRITE_BEGIN);
    sdMutex->unlock();
    if (f) {
        bufferPrinter = std::make_shared<BufferFilePrint>(f, sdMutex, 8192);
        bufferPrinters.push_back(bufferPrinter);
        streamSplitter.add(bufferPrinter);
    }else {
        bufferPrinter = std::make_shared<BufferFilePrint>(Serial, 8192);
        bufferPrinters.push_back(bufferPrinter);
    }
    delay(200);
    cmd_line_handler = std::make_shared<CommandLineHandler>(parser, Serial);
    xbeeCommandParserHandler = std::make_shared<CommandLineHandler>(xbeeCommandParser, Serial7);
    printHeader();
    /* check for CrashReport stored from previous run */
    if (CrashReport) {
        /* print info (hope Serial Monitor windows is open) */
        streamSplitter.print(CrashReport);
    }
    setupPROGMEM();

    //robot->addTarget(std::make_shared<PositionTarget<CalculatedQuadramp>>(robot, Position(1000,0), RampData(100, 200)));
    //robot->addTarget(std::make_shared<AngleTarget<CalculatedQuadramp>>(robot, Angle::fromDegrees(-90), RampData(45, 90)));
    //robot->addTarget(std::make_shared<PositionTarget<CalculatedQuadramp>>(robot, Position(1000, -400), RampData(200, 400)));
    /*
    robot->addTarget(std::make_shared<PositionTarget<CalculatedQuadramp>>(robot, Position(0,0), RampData(100, 200)));
    robot->addTarget(std::make_shared<PositionTarget<CalculatedQuadramp>>(robot, Position(1000,0), RampData(300,500)));
    //robot->addTarget(std::make_shared<PositionTarget<CalculatedQuadramp>>(robot, Position(0,0), RampData(100, 200)));
    robot->addTarget(std::make_shared<AngleTarget<CalculatedQuadramp>>(robot, Angle::fromDegrees(90), RampData(90, 180)));
    */
    /*
    G2Solve3Arc arc;
    Position end(1000, 200, Angle::fromDegrees(90), 0);
    Position end2(1500, 800, Angle::fromDegrees(0), 0);
    arc.build(robot->getCurrentPosition(), end);
    auto list = arc.getCurveList();
    arc.build(end, end2);
    list->addCurveList(arc.getCurveList());
    robot->addTarget(std::make_shared<CurveTarget<CalculatedQuadramp>>(robot, list, RampData(100, 200, 0)));
    */
    delay(5000);
    //robot->setControlDisabled(false);
    //robot->setEncoderToMotors();
    //robot->setControlDisabled(false);
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
    Threads::yield();
}