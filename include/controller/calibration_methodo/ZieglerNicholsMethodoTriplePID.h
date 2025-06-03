//
// Created by fogoz on 03/06/2025.
//

#ifndef PAMITEENSY_ZIEGLERNICHOLSMETHODOTRIPLEPID_H
#define PAMITEENSY_ZIEGLERNICHOLSMETHODOTRIPLEPID_H

#include "BaseCalibrationMethodo.h"

class PID;
class ZieglerNicholsMethodoTriplePID : public CalibrationMethodo{
protected:
    bool distance;
    std::shared_ptr<PID> pid;
    uint64_t index;
    uint64_t computeIndex;
public:
    ZieglerNicholsMethodoTriplePID(std::shared_ptr<BaseRobot> robot, std::shared_ptr<Mutex> mutex, bool distance);

    void start() override;

    void save() override;

    void stop() override;

    void openFile();


};


#endif //PAMITEENSY_ZIEGLERNICHOLSMETHODOTRIPLEPID_H
