//
// Created by fogoz on 23/04/2025.
//

#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"
#include <TeensyThreads.h>

inline void setCustomAnalog(uint8_t pin, uint32_t resolution, int value){
    static Threads::Mutex m;
    m.lock();
    uint32_t res = analogWriteResolution(resolution);
    analogWrite(pin, value);
    analogWriteResolution(res);
    m.unlock();
}

class Motor{
  public:
    virtual ~Motor() = default;

    virtual void setPWM(double pwm) = 0;

    virtual double getMaxPWM() = 0;

    virtual double getPWM() = 0;

    virtual bool isInversed() = 0;
};


#endif //MOTOR_H
