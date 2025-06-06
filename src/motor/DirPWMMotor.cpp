//
// Created by fogoz on 24/04/2025.
//

#include "motor/DirPWMMotor.h"

void DirPWMMotor::setPWM(double pwm) {
    double max_r = getMaxPWM();
    pwm = constrain(pwm, -max_r, max_r);
    this->current_pwm = pwm;
    pwm *= parameters->inversed ? -1 : 1;
    digitalWrite(dirPin, pwm > 0 ? LOW : HIGH);
    setCustomAnalog(pwmPin, parameters->resolution, (int)abs(pwm));
}

double DirPWMMotor::getMaxPWM() {
    return parameters->max_pwm * 0.8;
}

double DirPWMMotor::getPWM() {
    return this->current_pwm;
}

void DirPWMMotor::resetMaxPWM() {
    parameters->max_pwm = (1<<parameters->resolution) - 1;
}

bool DirPWMMotor::isInversed() {
    return parameters->inversed;
}

void DirPWMMotor::setInversed(bool inversed) {
    parameters->inversed = inversed;
}

void DirPWMMotor::setMaxPWM(double pwm) {
    parameters->max_pwm = min(pwm, (1<< parameters->resolution)) - 1;
}

DirPWMMotor::DirPWMMotor(uint8_t pwmPin, uint8_t dirPin, std::shared_ptr<MotorParameters> parameters) : pwmPin(pwmPin), dirPin(dirPin),
                                                                                               parameters(parameters) {
    pinMode(pwmPin, OUTPUT);
    pinMode(dirPin, OUTPUT);

    digitalWrite(dirPin, LOW);
    setCustomAnalog(pwmPin, parameters->resolution, 0);
    parameters->max_pwm = min((1<< parameters->resolution) - 1, parameters->max_pwm);
}
