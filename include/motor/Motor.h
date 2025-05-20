//
// Created by fogoz on 23/04/2025.
//

#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"
#include "ChRt.h"
#include "mutex"

#include "ArduinoJson.h"

struct MotorParameters {
    bool inversed = false;
    uint32_t resolution = 12;
    double max_pwm = (1 << resolution) - 1;
};

inline void setCustomAnalog(uint8_t pin, uint32_t resolution, int value){
    static mutex_t mutex;
    static bool init = false;
    if (!init) {
        chMtxObjectInit(&mutex);
        init = true;
    }

    chMtxLock(&mutex);
    uint32_t res = analogWriteResolution(resolution);
    analogWrite(pin, value);
    analogWriteResolution(res);
    chMtxUnlock(&mutex);
}

class Motor{
  public:
    virtual ~Motor() = default;

    virtual void setPWM(double pwm) = 0;

    virtual double getMaxPWM() = 0;

    virtual double getPWM() = 0;

    virtual void setMaxPWM(double pwm) = 0;

    virtual void resetMaxPWM() = 0;

    virtual bool isInversed() = 0;

    virtual void setInversed(bool inversed) = 0;
};

ARDUINOJSON_BEGIN_PUBLIC_NAMESPACE
template <>
struct Converter<MotorParameters> {
    static void toJson(const MotorParameters& src, JsonVariant dst) {
        dst["resolution"] = src.resolution;
        dst["max_pwm"] = src.max_pwm;
        dst["inversed"] = src.inversed;
    }

    static MotorParameters fromJson(JsonVariantConst src) {
        return {src["inversed"].as<bool>(), src["resolution"].as<uint32_t>(),src["max_pwm"].as<double>()};
    }

    static bool checkJson(JsonVariantConst src) {
        return src["inversed"].is<bool>() && src["resolution"].is<uint32_t>() && src["max_pwm"].is<double>();
    }
};

ARDUINOJSON_END_PUBLIC_NAMESPACE


#endif //MOTOR_H
