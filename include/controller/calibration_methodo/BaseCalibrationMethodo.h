//
// Created by fogoz on 03/06/2025.
//

#ifndef PAMITEENSY_BASECALIBRATIONMETHODO_H
#define PAMITEENSY_BASECALIBRATIONMETHODO_H
#include <SD.h>

#include "functional"
#include "utils/BufferFilePrint.h"
#include <utils/Mutex.h>
#include <robot/BaseRobot.h>
#include <memory>

class CalibrationMethodo{
protected:
    std::shared_ptr<BufferFilePrint> buffer = nullptr;
    std::shared_ptr<Mutex> bufferMutex = std::make_shared<Mutex>();
    File f;
    std::shared_ptr<Mutex> sdMutex;
    CallbackManager callbackManager;
    std::shared_ptr<BaseRobot> robot;
    std::shared_ptr<BaseController> previous_controller;

    std::shared_ptr<BaseController> currentController;
    std::vector<uint64_t> tasksId;

    bool done = false;

    virtual void openFile(const char* filename, uint16_t size=2048) {
        printerCleanup();
        lock_guard lg(sdMutex);
        lock_guard lg2(bufferMutex);
        SD.begin(BUILTIN_SDCARD);
        if (SD.exists(filename)) {
            SD.remove(filename);
        }
        f = SD.open(filename, FILE_WRITE_BEGIN);
        buffer = std::make_shared<BufferFilePrint>(f, sdMutex, size);
        streamSplitter.println("Assigned file to BufferFilePrint");
        bufferPrinters.add(buffer);
        streamSplitter.println("Added buffer to bufferPrinters list");
    }


    virtual void printerCleanup(){
        lock_guard lg(bufferMutex);
        if(buffer){
            bufferPrinters.remove(buffer);
        }
    }


public:
    virtual ~CalibrationMethodo() = default;

    CalibrationMethodo(std::shared_ptr<BaseRobot> robot, std::shared_ptr<Mutex> sdMutex) : sdMutex(sdMutex), robot(robot){

    }
    virtual void start(){
        robot->setControlDisabled(true);
        robot->setDoneAngular(true);
        robot->setDoneDistance(true);
        previous_controller = robot->getController();
    }

    virtual void stop(){
        robot->getEventEndOfComputeNotifier()->wait();
        robot->setControlDisabled(true);
        robot->setDoneAngular(true);
        robot->setDoneDistance(true);
        printerCleanup();
        for(auto& a : tasksId){
            scheduler->deleteTaskId(a);
        }
        tasksId.clear();
        done = true;
        robot->getLeftMotor()->setPWM(0);
        robot->getRightMotor()->setPWM(0);
        robot->setController(previous_controller);
    }

    virtual uint64_t setEndOfStepCallback(std::function<void()> callback){
        return callbackManager.addCallback(callback);
    }

    virtual void removeEndOfStepCallback(uint64_t callback) {
        callbackManager.removeCallback(callback);
    }

    [[nodiscard]] bool isDone() const {
        return done;
    }

    virtual void save() = 0;

    virtual void awaitBeforeDestruction() {
        lock_guard lg(bufferMutex);
        if (bufferPrinter != nullptr)
            bufferPrinter->flush();
    }

    virtual void printStatus(Stream & stream) = 0;
};
class PAMIRobot;
class BasicController;

void writeControllerToBuffer(std::shared_ptr<BufferFilePrint> buffer, std::shared_ptr<BasicController> controller, std::shared_ptr<BaseRobot> robot);

void writeControllerTypeToBuffer(std::shared_ptr<BufferFilePrint> buffer, std::shared_ptr<BasicController> controller);

#endif //PAMITEENSY_BASECALIBRATIONMETHODO_H
