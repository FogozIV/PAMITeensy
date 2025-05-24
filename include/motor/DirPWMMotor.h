//
// Created by fogoz on 24/04/2025.
//

#ifndef DIRPWMMOTOR_H
#define DIRPWMMOTOR_H

#include <memory>

#include "Motor.h"
#include "Arduino.h"

/**
 * @brief Direction and PWM based motor control implementation
 * 
 * This class implements motor control using:
 * - PWM signal for speed control
 * - Direction pin for rotation direction
 * - Configurable PWM parameters
 * - Thread-safe operation
 * 
 * The motor is controlled by:
 * - PWM duty cycle (-1.0 to 1.0)
 * - Direction pin (HIGH/LOW)
 * - Configurable PWM resolution
 */
class DirPWMMotor : public Motor{
    uint8_t pwmPin;    ///< PWM output pin
    uint8_t dirPin;    ///< Direction control pin

    double current_pwm = 0;

    std::shared_ptr<MotorParameters> parameters;
public:

    /**
     * @brief Constructs a new direction/PWM motor controller
     * 
     * @param pwmPin PWM output pin number
     * @param dirPin Direction control pin number
     * @param parameters Motor configuration parameters
     */
    DirPWMMotor(uint8_t pwmPin, uint8_t dirPin, std::shared_ptr<MotorParameters> parameters);

    /**
     * @brief Sets the motor speed
     * 
     * Sets the motor speed using PWM. The value is clamped to
     * the range [-1.0, 1.0]. The sign determines the direction:
     * - Positive: Forward rotation
     * - Negative: Reverse rotation
     * 
     * @param pwm PWM value (-1.0 to 1.0)
     */
    void setPWM(double pwm) override;

    /**
     * @brief Gets the maximum allowed PWM value
     * @return double Maximum PWM value
     */
    double getMaxPWM() override;

    /**
     * @brief Gets the current PWM value
     * @return double Current PWM value (-1.0 to 1.0)
     */
    double getPWM() override;

    /**
     * @brief Resets PWM limits to default
     */
    void resetMaxPWM() override;

    /**
     * @brief Checks if motor direction is inversed
     * @return bool True if direction is inversed
     */
    bool isInversed() override;

    /**
     * @brief Sets motor direction inversion
     * @param inversed True to inverse direction
     */
    void setInversed(bool inversed) override;

    /**
     * @brief Sets maximum allowed PWM value
     * @param pwm New maximum PWM value
     */
    void setMaxPWM(double pwm) override;
};



#endif //DIRPWMMOTOR_H
