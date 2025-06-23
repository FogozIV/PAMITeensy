//
// Created by fogoz on 03/06/2025.
//

#ifndef PAMITEENSY_ZIEGLERNICHOLSMETHODOTRIPLEPID_H
#define PAMITEENSY_ZIEGLERNICHOLSMETHODOTRIPLEPID_H

#include "BaseCalibrationMethodo.h"
#include "controller/SimpleTripleBasicController.h"
#include "utils/OscillationTracker.h"

#define ZIEGLER_NICHOLS_PID_CHOICE \
    PID_CHOICE(P_TYPE,                 0.50, 0.00, 0.00) \
    PID_CHOICE(PI_TYPE,                0.45, 0.54, 0.00) \
    PID_CHOICE(PD_TYPE,                0.80, 0.00, 0.10) \
    PID_CHOICE(CLASSICAL_PID_TYPE,    0.60, 1.20, 0.075) \
    PID_CHOICE(PESSEN_INTEGRAL_PID_TYPE, 0.70, 1.75, 0.105) \
    PID_CHOICE(SOME_OVERSHOT,         0.33, 0.66, 0.11) \
    PID_CHOICE(NO_OVERSHOT,           0.20, 0.40, 0.066) \

#define PID_CHOICE(value, ...) value,

namespace ZieglerPIDChoice {
    enum Choice{
        ZIEGLER_NICHOLS_PID_CHOICE
    };
}
#undef PID_CHOICE

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

    double finalGain = 0;
    double finalPeriod = 0;

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

    void setPIDTo(ZieglerPIDChoice::Choice choice);
};


#endif //PAMITEENSY_ZIEGLERNICHOLSMETHODOTRIPLEPID_H
