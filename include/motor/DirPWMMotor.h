//
// Created by fogoz on 24/04/2025.
//

#ifndef DIRPWMMOTOR_H
#define DIRPWMMOTOR_H

#include "Motor.h"
#include "Arduino.h"
class DirPWMMotor : public Motor{
    uint8_t pwmPin;
    uint8_t dirPin;

    bool inversed = false;
    double current_pwm = 0;
    uint32_t resolution;
public:

    DirPWMMotor(uint8_t pwmPin, uint8_t dirPin, bool inversed=false, uint32_t resolution=12);

    void setPWM(double pwm) override;

    double getMaxPWM() override;

    double getPWM() override;

    bool isInversed() override;

    void setInversed(bool inversed) override;
};



#endif //DIRPWMMOTOR_H
