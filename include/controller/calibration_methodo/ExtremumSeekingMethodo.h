//
// Created by fogoz on 05/06/2025.
//

#ifndef EXTREMUMSEEKINGMETHODO_H
#define EXTREMUMSEEKINGMETHODO_H
#include "BaseCalibrationMethodo.h"


class ExtremumSeekingMethodo : public CalibrationMethodo {
public:
    ExtremumSeekingMethodo(const std::shared_ptr<BaseRobot> &robot, const std::shared_ptr<Mutex> &sdMutex);

    void save() override;

    void printStatus(Stream &stream) override;

    void start() override;

    void stop() override;
    virtual ~ExtremumSeekingMethodo() = default;
};



#endif //EXTREMUMSEEKINGMETHODO_H
