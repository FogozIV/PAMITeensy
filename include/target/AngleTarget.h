//
// Created by fogoz on 08/05/2025.
//

#ifndef ANGLETARGET_H
#define ANGLETARGET_H
#include "BaseTarget.h"
#include "ramp/Ramp.h"
#include "utils/Angle.h"


class AngleTarget : public BaseTarget{
    bool done = false;
    std::shared_ptr<Ramp> ramp;
    RampData ramp_data;
    Angle target_angle;
public:
    bool is_done() override;

    void init() override;

    void process() override;

    void reInitAfterStop() override;
};



#endif //ANGLETARGET_H
