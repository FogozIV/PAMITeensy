//
// Created by fogoz on 26/05/2025.
//

#ifndef PAMITEENSY_RELATIVEPOSITIONTARGET_H
#define PAMITEENSY_RELATIVEPOSITIONTARGET_H
#include "PositionTarget.h"

#include <utility>
template<typename Ramp>
class RelativePositionTarget : public PositionTarget<Ramp>{
protected:
    Position relative;
public:
    RelativePositionTarget(std::shared_ptr<BaseRobot> robot, Position relative, RampData rampdata) : PositionTarget<Ramp>(robot, Position(), rampdata){
        this->relative = std::move(relative);
    }

    void init() override{
        this->pos = this->robot->getCurrentPosition().offsetRelative(this->relative);
        PositionTarget<Ramp>::init();
    }
};

#endif //PAMITEENSY_RELATIVEPOSITIONTARGET_H
