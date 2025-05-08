//
// Created by fogoz on 24/04/2025.
//

#ifndef DIRPWMMOTOR_H
#define DIRPWMMOTOR_H

#include <memory>

#include "Motor.h"
#include "Arduino.h"
class DirPWMMotor : public Motor{
    uint8_t pwmPin;
    uint8_t dirPin;

    double current_pwm = 0;

    std::shared_ptr<MotorParameters> parameters;
public:

    DirPWMMotor(uint8_t pwmPin, uint8_t dirPin, std::shared_ptr<MotorParameters> parameters);

    void setPWM(double pwm) override;

    double getMaxPWM() override;

    double getPWM() override;

    void resetMaxPWM() override;

    bool isInversed() override;

    void setInversed(bool inversed) override;

    void setMaxPWM(double pwm) override;
};



#endif //DIRPWMMOTOR_H
