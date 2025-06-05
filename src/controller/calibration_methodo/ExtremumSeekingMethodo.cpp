//
// Created by fogoz on 05/06/2025.
//

#include <controller/calibration_methodo/ExtremumSeekingMethodo.h>

ExtremumSeekingMethodo::ExtremumSeekingMethodo(const std::shared_ptr<BaseRobot> &robot,
    const std::shared_ptr<Mutex> &sdMutex): CalibrationMethodo(robot, sdMutex) {
}

void ExtremumSeekingMethodo::save() {
}

void ExtremumSeekingMethodo::printStatus(Stream &stream) {
}

void ExtremumSeekingMethodo::start() {
    CalibrationMethodo::start();

}

void ExtremumSeekingMethodo::stop() {
    CalibrationMethodo::stop();

}
