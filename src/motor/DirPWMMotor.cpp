//
// Created by fogoz on 24/04/2025.
//

#include "motor/DirPWMMotor.h"

void DirPWMMotor::setPWM(double pwm) {
    double max_r = getMaxPWM();
    pwm = max(min(pwm, max_r), -max_r);
    this->current_pwm = pwm;
    pwm *= inversed ? -1 : 1;
    digitalWrite(dirPin, pwm > 0 ? LOW : HIGH);
    setCustomAnalog(pwmPin, resolution, (int)abs(pwm));
}

double DirPWMMotor::getMaxPWM() {
    return (2 << resolution) - 1;
}

double DirPWMMotor::getPWM() {
    return this->current_pwm;
}

bool DirPWMMotor::isInversed() {
    return inversed;
}

void DirPWMMotor::setInversed(bool inversed) {
    this->inversed = inversed;
}

DirPWMMotor::DirPWMMotor(uint8_t pwmPin, uint8_t dirPin, bool inversed, uint32_t resolution) : pwmPin(pwmPin), dirPin(dirPin),
                                                                                               inversed(inversed), resolution(resolution) {
    pinMode(pwmPin, OUTPUT);
    pinMode(dirPin, OUTPUT);

    digitalWrite(dirPin, LOW);
    setCustomAnalog(pwmPin, resolution, 0);
}
