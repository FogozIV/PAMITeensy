//
// Created by fogoz on 03/06/2025.
//

#ifndef PAMITEENSY_ZIEGLERNICHOLSMETHODOTRIPLEPID_H
#define PAMITEENSY_ZIEGLERNICHOLSMETHODOTRIPLEPID_H

#include "BaseCalibrationMethodo.h"
#include "controller/SimpleTripleBasicController.h"
#include "utils/OscillationTracker.h"

class PID;
class ZieglerNicholsMethodoTriplePID : public CalibrationMethodo{
protected:
    bool distance;
    std::shared_ptr<PID> pid;
    uint64_t index{};
    uint64_t computeIndex{};
    std::shared_ptr<OscillationTracker> oscTracker;
    double initialValue = 2;
    double target;
    bool forward = false;
    double multiplier = 1.2;
    TripleController::SpeedMode speed;
public:
    ZieglerNicholsMethodoTriplePID(std::shared_ptr<BaseRobot> robot, std::shared_ptr<Mutex> mutex, bool distance, TripleController::SpeedMode speed);

    void start() override;

    void save() override;

    void stop() override;

    void setInitialValue(double value);

    void openFile();

    void printStatus(Stream &stream) override;

    void setTarget(double value);

    void setMultiplier(double multiplier);
};


#endif //PAMITEENSY_ZIEGLERNICHOLSMETHODOTRIPLEPID_H
