//
// Created by fogoz on 03/06/2025.
//

#ifndef PAMITEENSY_BASECALIBRATIONMETHODO_H
#define PAMITEENSY_BASECALIBRATIONMETHODO_H
#include "functional"
#include "utils/BufferFilePrint.h"
#include <utils/Mutex.h>
#include <robot/BaseRobot.h>

class CalibrationMethodo{
protected:
    std::shared_ptr<BufferFilePrint> buffer = nullptr;
    File f;
    std::shared_ptr<Mutex> mutex;
    std::function<void()> endOfStep = nullptr;
    std::shared_ptr<BaseRobot> robot;
    std::shared_ptr<BaseController> previous_controller;

    std::shared_ptr<BaseController> currentController;

public:

    CalibrationMethodo(std::shared_ptr<BaseRobot> robot, std::shared_ptr<Mutex> sdMutex) : mutex(sdMutex), robot(robot){

    }
    virtual void start(){
        robot->setControlDisabled(true);
        robot->setDoneAngular(true);
        robot->setDoneDistance(true);
        previous_controller = robot->getController();
    }

    virtual void stop(){
        printerCleanup();
    }

    virtual void printerCleanup(){
        if(buffer){
            bufferPrinters.remove(buffer);
            f.close();
        }
    }

    virtual void setEndOfStepCallback(std::function<void()> callback){
        endOfStep = callback;
    }

    virtual void save() = 0;
};

#endif //PAMITEENSY_BASECALIBRATIONMETHODO_H
