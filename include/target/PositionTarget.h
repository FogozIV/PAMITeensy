//
// Created by fogoz on 07/05/2025.
//

#ifndef POSITIONTARGET_H
#define POSITIONTARGET_H
#include "BaseTarget.h"
#include "ramp/Ramp.h"
#include "robot/BaseRobot.h"

template<typename T>
class PositionTarget : public BaseTarget{
    bool done = false;
    Position pos;
    std::shared_ptr<Ramp> ramp = nullptr;
    RampData ramp_data;

public:
    PositionTarget(std::shared_ptr<BaseRobot> baseRobot, Position pos, RampData rampData);

    bool is_done() override;

    void init() override;

    void process() override;

    void on_done() override;

    void reInitAfterStop() override;
};

#include "PositionTarget.tpp"



#endif //POSITIONTARGET_H
