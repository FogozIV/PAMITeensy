//#define _TEENSY41_ASYNC_TCP_LOGLEVEL_     5

#include "ChRt.h"
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
#include "curves/ClothoidCurve.h"
#include "utils/StreamSplitter.h"

#include "curves/CurveList.h"
#include "utils/TaskScheduler.h"

//#define ENABLE_WEB_SERVER_OTA

std::shared_ptr<PAMIRobot> robot;
std::shared_ptr<CommandLineHandler> cmd_line_handler;
std::shared_ptr<CommandLineHandler> xbeeCommandParserHandler;
std::shared_ptr<BufferFilePrint> bufferPrinter;
std::shared_ptr<TaskScheduler> scheduler;
std::vector<std::shared_ptr<BufferFilePrint> > bufferPrinters;
std::map<uint16_t, std::shared_ptr<CustomAsyncClient> > customAsyncClientMap;

File f;
std::shared_ptr<AsyncServer> server;
using namespace std::chrono;
bool flashing_process = false;

CommandParser parser;
CommandParser xbeeCommandParser;
PacketHandler packetHandler;

#define WORKING_AREAS \
    AREA(waHandleRobotUpdate, 8192)\
    AREA(waHandleCommandLine, 8192)\
    AREA(waHandleBufferPrinters, 512)\
    AREA(waDEBUG, 1024) \
    AREA(__main_thread_stack_base__, 1024)

#define AREA(name, size)\
    THD_WORKING_AREA(name, size);
WORKING_AREAS
#undef AREA

static THD_FUNCTION(handle_robot_update, arg) {
    (void) arg;
    chRegSetThreadName("robot_update_handler");
    const systime_t period = TIME_MS2I(5);
    systime_t lastWakeTime = chVTGetSystemTime();
    streamSplitter.println("Starting up scheduler");
    scheduler->init();
    streamSplitter.println("Starting up scheduler done");
    while (true) {

        robot->compute();
        lastWakeTime += period;
        if (chVTGetSystemTime() > lastWakeTime) {
            lastWakeTime = chVTGetSystemTime();  // Reset
        }
        chThdSleepUntil(lastWakeTime);
    }
}

static THD_FUNCTION(handle_command_line, arg) {
    (void) arg;
    const systime_t period = TIME_MS2I(5);
    systime_t lastWakeTime = chVTGetSystemTime();
    chRegSetThreadName("command_line_handler");

    while (true) {
        cmd_line_handler->handle_commandline();
        xbeeCommandParserHandler->handle_commandline();
        lastWakeTime += period;
        if (chVTGetSystemTime() > lastWakeTime) {
            lastWakeTime = chVTGetSystemTime();  // Reset
        }
        chThdSleepUntil(lastWakeTime);
    }
}

static THD_FUNCTION(handle_buffer_printers, arg) {
    (void) arg;
    const systime_t period = TIME_MS2I(100);
    systime_t lastWakeTime = chVTGetSystemTime();
    chRegSetThreadName("buffer_printers_handler");
    while (true) {
        for (auto &buffered: bufferPrinters) {
            buffered->flush();
        }
        lastWakeTime += period;
        if (chVTGetSystemTime() > lastWakeTime) {
            lastWakeTime = chVTGetSystemTime();  // Reset
        }
        chThdSleepUntil(lastWakeTime);
    }
}
void printThreadStates() {
    thread_t *tp = chRegFirstThread();
    do {
        Serial.printf("Thread: %s, state: %d\n", 
                     tp->name ? tp->name : "unnamed", 
                     tp->state);
        tp = chRegNextThread(tp);
    } while (tp != NULL);
}
void printThreadStats() {
    thread_t *tp = chRegFirstThread();

    Serial.println("Thread Statistics:");
    Serial.println("Addr       | Name        | Time");

    while (tp != nullptr) {
        const char *name = tp->name ? tp->name : "(unnamed)";

        Serial.printf("%08lx | %-12s | %lu\n",
                      (uint32_t)tp,
                      name ? name : "(unnamed)",
                      (unsigned long)(tp->stats.cumulative));

        tp = chRegNextThread(tp);
    }
}
size_t getSize(uint8_t* wabase) {
#define AREA(name, size)\
    if(wabase == (uint8_t*)name){\
        return sizeof(name);\
    }
    WORKING_AREAS
    return 0;

}


size_t getStackUsed(thread_t* tp) {
    uint8_t *wabase = (uint8_t *)tp->wabase;
    size_t size = getSize(wabase);

    size_t used = 0;
    for (size_t i = 0; i < size; i++) {
        if (wabase[i] != CH_DBG_STACK_FILL_VALUE) {
            used = size - i;
            break;
        }
    }
    return used;
}
void printStackUsage() {
    thread_t *tp = chRegFirstThread();
    Serial.println("Thread Stack Usage:");

    while (tp != nullptr) {
        const char *name = tp->name ? tp->name : "(unnamed)";
        size_t used = getStackUsed(tp);
        Serial.printf("%-12s | Stack used: %lu / %lu bytes\n",
                      name ? name : "(unnamed)",
                      used,
                      getSize((uint8_t*)tp->wabase));

        tp = chRegNextThread(tp);
    }
}
static void debugThread(void *) {
    while (true) {
        printThreadStats();
        printStackUsage();
        chThdSleepMilliseconds(2000);
    }
}

void setupThreads() {
    chThdCreateStatic(waDEBUG, sizeof(waDEBUG), HIGHPRIO, debugThread, NULL);
    streamSplitter.println(("Initialising robot updater"));
    auto th = chThdCreateStatic(waHandleRobotUpdate, sizeof(waHandleRobotUpdate), NORMALPRIO + 2, handle_robot_update, NULL);
    if (th == nullptr) {
        streamSplitter.println("Failed to create robot updater!");
    }
    streamSplitter.println("Initialising command line updater");
    th = chThdCreateStatic(waHandleCommandLine, sizeof(waHandleCommandLine), NORMALPRIO, handle_command_line, NULL);
    if (th == nullptr) {
        streamSplitter.println("Failed to create command liner handler!");
    }
    streamSplitter.println("Initialising SD Card task");
    th = chThdCreateStatic(waHandleBufferPrinters, sizeof(waHandleBufferPrinters), NORMALPRIO, handle_buffer_printers, NULL);
    if (th == nullptr) {
        streamSplitter.println("Failed to create command liner handler!");
    }
    digitalWriteFast(LED_BUILTIN, LOW);
}

void setup() {
    for (int i = 0; i < 42; i++) {
        pinMode(i, INPUT);
    }
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWriteFast(LED_BUILTIN, HIGH);
    chSysInit();
    Serial.begin(1'000'000);
    Serial7.begin(115200, SERIAL_8N1);
    SD.begin(BUILTIN_SDCARD);
    f = SD.open((String(rtc_get()) + ".txt").c_str(), FILE_WRITE_BEGIN);
    if (f) {
        bufferPrinter = std::make_shared<BufferFilePrint>(f, 8192);
        bufferPrinters.push_back(bufferPrinter);
    }
    cmd_line_handler = std::make_shared<CommandLineHandler>(parser, Serial);
    xbeeCommandParserHandler = std::make_shared<CommandLineHandler>(xbeeCommandParser, Serial7);
    printHeader();
    /* check for CrashReport stored from previous run */
    if (CrashReport) {
        /* print info (hope Serial Monitor windows is open) */
        streamSplitter.print(CrashReport);
    }
    auto status = setupEthernet();
    if (status == CustomEthernetStatus::OK) {
        streamSplitter.println("Ethernet initialized");
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

    streamSplitter.printf(("Hello world ! Welcome to the teensy, it was compiled the %s at %s \r\n"), __DATE__,
                          __TIME__);
    /*
    * Create the robot and initialize it, this will also create the motors and the servos
    */
    streamSplitter.println(("Creating robot"));
    robot = std::make_shared<PAMIRobot>();
    robot->init();
    /*
     * Register the commands that will be available in the command line
     */
    streamSplitter.println(("Registering commands"));
    registerCommands(xbeeCommandParser, robot);
    registerCommands(parser, robot);
    robot->registerCommands(xbeeCommandParser);
    robot->registerCommands(parser);
    streamSplitter.println(("Registering commands done"));
    /*
     * Disable the control of the robot, we will use the command line to control the robot
     */
    robot->setControlDisabled(true);
    robot->addTarget(
        std::make_shared<PositionTarget<CalculatedQuadramp> >(robot, Position(1000, 0), RampData(100, 200)));
    robot->addTarget(
        std::make_shared<AngleTarget<CalculatedQuadramp> >(robot, Angle::fromDegrees(180), RampData(45, 90)));
    robot->addTarget(std::make_shared<PositionTarget<CalculatedQuadramp> >(robot, Position(0, 0), RampData(100, 200)));
    scheduler = std::make_shared<TaskScheduler>();
    chBegin([] {
        setupThreads();  // Inline lambda to ensure capture context is valid
    });
}


void loop() {
    //printThreadStates();
    chThdSleepMilliseconds(1000);
}
