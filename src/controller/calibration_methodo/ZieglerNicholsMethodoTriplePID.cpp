//
// Created by fogoz on 03/06/2025.
//

#include "controller/calibration_methodo/ZieglerNicholsMethodoTriplePID.h"
#include <controller/SimpleTripleBasicController.h>
#include "basic_controller/PID.h"
#include "SD.h"
#include <utils/Mutex.h>
#include <utils/TaskScheduler.h>

void ZieglerNicholsMethodoTriplePID::start() {
    CalibrationMethodo::start();
    pid = std::make_shared<PID>(robot, 2, 0,0,1000);
    currentController = std::make_shared<SimpleTripleBasicController>(robot, pid, pid, pid, std::make_shared<TripleBasicParameters>());

    if(distance){
        robot->setTranslationalPosition(0);
        robot->setTranslationalTarget(400);
        robot->setDoneDistance(false);
        computeIndex = robot->addEndComputeHook([this](){
            buffer->printf("%f; %f; %f\r\n", robot->getTranslationalTarget(), robot->getTranslationalPosition(), robot->getTranslationalPosition() - robot->getTranslationalTarget());
        });
    }else{
        robot->setRotationalPosition(AngleConstants::ZERO);
        robot->setRotationalTarget(AngleConstants::LEFT);
        robot->setDoneAngular(false);
        computeIndex = robot->addEndComputeHook([this](){
            buffer->printf("%f; %f; %f\r\n", robot->getRotationalTarget().toDegrees(), robot->getRotationalPosition().toDegrees(), (robot->getRotationalPosition() - robot->getRotationalTarget()).toDegrees());
        });
    }
    index = scheduler->addTask(seconds(10), [this](){
        robot->setControlDisabled(true);
        robot->getEventEndOfComputeNotifier().wait(); //dangerous because it can hang a thread that's why we use a threadpool

        pid->getKpRef() * 1.2;
        robot->setControlDisabled(false);
    }, seconds(10));

}

void ZieglerNicholsMethodoTriplePID::save() {

}

ZieglerNicholsMethodoTriplePID::ZieglerNicholsMethodoTriplePID(std::shared_ptr<BaseRobot> robot, std::shared_ptr<Mutex> mutex, bool distance) : CalibrationMethodo(robot, mutex), distance(distance) {

}

void ZieglerNicholsMethodoTriplePID::stop() {
    CalibrationMethodo::stop();
    scheduler->deleteTaskId(index);
    robot->removeEndComputeHook(computeIndex);

}

void ZieglerNicholsMethodoTriplePID::openFile() {
    if(buffer){
        bufferPrinters.remove(buffer);
        f.close();
    }
    lock_guard lg(mutex);
    SD.begin(BUILTIN_SDCARD);
    String data = String("ZieglerNicholsKP=") + String(pid->getKp()) + String(".txt");
    f = SD.open(data.c_str(), FILE_WRITE_BEGIN);
    buffer = std::make_shared<BufferFilePrint>(f, mutex, 2048);
    bufferPrinters.add(buffer);
}
