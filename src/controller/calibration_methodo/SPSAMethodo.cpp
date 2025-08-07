//
// Created by fogoz on 06/08/2025.
//

#include <controller/calibration_methodo/SPSAMethodo.h>

SPSAMethodo::SPSAMethodo(std::shared_ptr<BaseRobot> robot, std::shared_ptr<Mutex> sdMutex, ESCType::ESC typeOfControl) : CalibrationMethodo(robot, sdMutex), typeOfControl(typeOfControl) {
}

void SPSAMethodo::save() {
}

void SPSAMethodo::printStatus(Stream &stream) {
}

void SPSAMethodo::start() {
}

void SPSAMethodo::stop() {
}

void SPSAMethodo::launchStage() {
    logGains.clear();
    gainsValues = robot->getController()->getGains(typeOfControl);
    for (auto gain : gainsValues) {
        logGains.push_back(log(gain));
    }
    random(0, 2);

}

void SPSAMethodo::cleanupStage() {
}
