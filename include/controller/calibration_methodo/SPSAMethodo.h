//
// Created by fogoz on 06/08/2025.
//

#ifndef PAMITEENSY_SPSAMETHODO_H
#define PAMITEENSY_SPSAMETHODO_H

#include <controller/calibration_methodo/BaseCalibrationMethodo.h>
#include <vector>
#include <controller/calibration_methodo/ESCType.h>

class SPSAMethodo : public CalibrationMethodo{
    std::vector<double> gainsValues;
    std::vector<double> logGains;
    std::vector<int8_t> perturbation;
    ESCType::ESC typeOfControl;
public:
    SPSAMethodo(std::shared_ptr<BaseRobot> robot, std::shared_ptr<Mutex> sdMutex, ESCType::ESC typeOfControl);

    void save() override;

    void printStatus(Stream &stream) override;

    void start() override;

    void stop() override;

    void launchStage();

    void cleanupStage();
};


#endif //PAMITEENSY_SPSAMETHODO_H