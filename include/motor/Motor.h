//
// Created by fogoz on 23/04/2025.
//

#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"
#include "utils/Mutex.h"

#include "ArduinoJson.h"

/**
 * @brief Configuration parameters for motor control
 * 
 * This structure holds the basic configuration parameters for a motor,
 * including direction, PWM resolution, and maximum PWM value.
 */
struct MotorParameters {
    bool inversed = false;      ///< Motor direction inversion flag
    uint32_t resolution = 12;   ///< PWM resolution in bits
    double max_pwm = (1 << resolution) - 1;  ///< Maximum PWM value based on resolution
};

/**
 * @brief Thread-safe analog write function with custom resolution
 * 
 * This function provides a thread-safe way to write analog values with
 * a specific resolution to a pin.
 * 
 * @param pin The Arduino pin number to write to
 * @param resolution The desired PWM resolution in bits
 * @param value The analog value to write
 */
inline void setCustomAnalog(uint8_t pin, uint32_t resolution, int value){
    static Mutex m;
    m.lock();
#ifdef ESP32
    analogWriteResolution(resolution);
    analogWrite(pin, value);
#endif
#ifdef TEENSY41
    uint32_t res = analogWriteResolution(resolution);
    analogWrite(pin, value);
    analogWriteResolution(res);
#endif
    m.unlock();
}

/**
 * @brief Abstract base class for motor control
 * 
 * This class defines the interface for controlling motors in the system.
 * It provides methods for PWM control, direction control, and motor configuration.
 */
class Motor{
  public:
    virtual ~Motor() = default;

    /**
     * @brief Sets the PWM value for the motor
     * @param pwm PWM value between -1.0 and 1.0
     */
    virtual void setPWM(double pwm) = 0;

    /**
     * @brief Gets the maximum allowed PWM value
     * @return double Maximum PWM value
     */
    virtual double getMaxPWM() = 0;

    /**
     * @brief Gets the current PWM value
     * @return double Current PWM value
     */
    virtual double getPWM() = 0;

    /**
     * @brief Sets the maximum allowed PWM value
     * @param pwm New maximum PWM value
     */
    virtual void setMaxPWM(double pwm) = 0;

    /**
     * @brief Resets the maximum PWM value to default
     */
    virtual void resetMaxPWM() = 0;

    /**
     * @brief Checks if the motor direction is inversed
     * @return bool True if motor direction is inversed
     */
    virtual bool isInversed() = 0;

    /**
     * @brief Sets the motor direction inversion
     * @param inversed True to inverse motor direction
     */
    virtual void setInversed(bool inversed) = 0;
};

/**
 * @brief ArduinoJson converter specialization for MotorParameters
 * 
 * This specialization allows MotorParameters to be serialized to
 * and deserialized from JSON format.
 */
ARDUINOJSON_BEGIN_PUBLIC_NAMESPACE
template <>
struct Converter<MotorParameters> {
    /**
     * @brief Converts MotorParameters to JSON
     * @param src Source MotorParameters object
     * @param dst Destination JSON variant
     */
    static void toJson(const MotorParameters& src, JsonVariant dst) {
        dst["resolution"] = src.resolution;
        dst["max_pwm"] = src.max_pwm;
        dst["inversed"] = src.inversed;
    }

    /**
     * @brief Creates MotorParameters from JSON
     * @param src Source JSON variant
     * @return MotorParameters Constructed parameters
     */
    static MotorParameters fromJson(JsonVariantConst src) {
        return {src["inversed"].as<bool>(), src["resolution"].as<uint32_t>(),src["max_pwm"].as<double>()};
    }

    /**
     * @brief Validates JSON format for MotorParameters
     * @param src JSON variant to check
     * @return bool True if JSON is valid for MotorParameters
     */
    static bool checkJson(JsonVariantConst src) {
        return src["inversed"].is<bool>() && src["resolution"].is<uint32_t>() && src["max_pwm"].is<double>();
    }
};
ARDUINOJSON_END_PUBLIC_NAMESPACE


#endif //MOTOR_H
