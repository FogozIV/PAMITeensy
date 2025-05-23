//
// Created by fogoz on 24/04/2025.
//

#ifndef PAMITEENSY_BASECONTROLLER_H
#define PAMITEENSY_BASECONTROLLER_H
#include "memory"
#include "Arduino.h"

inline double applyMaxMin(double value, double max_min){
    return constrain(value, -max_min, max_min);
}

inline double applySpeedMin(double value, double speed_min){
    if(value > 0){
        value += speed_min;
    }else if(value < 0){
        value -= speed_min;
    }
    return value;
}

class BaseController{
public:
    virtual void compute() = 0;
    virtual void reset(bool correct_error=false) = 0;
};

#endif //PAMITEENSY_BASECONTROLLER_H
