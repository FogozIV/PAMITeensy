
#include <Arduino.h>

#include "robot/PAMIRobot.h"
#include "CommandParser.h"
#include "TeensyThreads.h"
#include "utils/RegisterCommands.h"
#include "utils/BufferFilePrint.h"
#include "ramp/DynamicQuadramp.h"
#include "utils/HeaderPrint.h"
#include "QNEthernet.h"
#include "utils/SetupEthernet.h"
#include "Teensy41_AsyncTCP.h"
std::shared_ptr<PAMIRobot> robot;
std::shared_ptr<std::thread> usb_command_line;
std::shared_ptr<std::thread> sd_writer;
std::shared_ptr<std::thread> robot_update;
std::vector<std::shared_ptr<BufferFilePrint>> bufferPrinters;


using namespace std::chrono;

CommandParser parser;
std::shared_ptr<EthernetServer> server = nullptr;
std::vector<std::shared_ptr<EthernetClient>> clients;
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


void setup() {
    Serial.begin(1000000);
    /* check for CrashReport stored from previous run */
    if (CrashReport) {
        /* print info (hope Serial Monitor windows is open) */
        Serial.print(CrashReport);
    }
    printHeader();
    CustomEthernetStatus status = setupEthernet();
    Serial.print(status);
    if (status == CustomEthernetStatus::OK) {
        Serial.println("Ethernet initialized");
        server = std::make_shared<EthernetServer>(23);
        server->begin();
        /*
        server = std::make_shared<AsyncServer>(23);
        server->onClient([](void* _, AsyncClient* client) {
            Serial.println("Client connected");
            client->onData([](void* _, AsyncClient* client, void* data, size_t len) {
                Serial.print("Received data: ");
                Serial.write(static_cast<uint8_t *>(data), len);
            }, nullptr);
        }, nullptr);
        server->begin();*/
    }
    Serial.printf("Hello world ! Welcome to the teensy, it was compiled the %s at %s \r\n", __DATE__, __TIME__);
    Serial.println("FogozIV was here");
    Serial.println("To Implement OTA");
    robot = std::make_shared<PAMIRobot>();
    robot->init(robot);
    registerCommands(parser, robot);

    threads.setDefaultStackSize(10000);
    threads.setDefaultTimeSlice(10);
    threads.setSliceMicros(10);

    usb_command_line = std::make_shared<std::thread>(handle_usb_command);
    usb_command_line->detach();

    sd_writer = std::make_shared<std::thread>(handle_sd_card);
    sd_writer->detach();

    robot_update = std::make_shared<std::thread>(handle_robot_update);
    robot_update->detach();

    robot->setControlDisabled(true);

}
void loop() {
    if (server!= nullptr) {
        EthernetClient client = server->accept();
        if (client) {
            Serial.println("Ethernet client connected");
            clients.emplace_back(std::make_shared<EthernetClient>(std::move(client)));
        }
    }
    for (auto it = clients.begin(); it != clients.end(); ) {
        auto client = it->get();
        if (!client->connected()) {
            Serial.println("Ethernet client disconnected");
            it = clients.erase(it);
        }else {
            ++it;
        }
    }
}