//
// Created by fogoz on 22/05/2025.
//

#ifndef PAMITEENSY_CONFIG_H
#define PAMITEENSY_CONFIG_H
#include <Arduino.h>
#include "ThreadSafeSerial.h"

#define LEFT_DIR 41
#define RIGHT_DIR 40

#define LEFT_PWM 36
#define RIGHT_PWM 33

#define Serial    SerialTS
#define Serial1   Serial1TS
#define Serial2   Serial2TS
#define Serial3   Serial3TS
#define Serial4   Serial4TS
#define Serial5   Serial5TS
#define Serial6   Serial6TS
#define Serial7   Serial7TS
#define Serial8   Serial8TS

#define CATCH_IN_LAMBDA(caller, function) [caller]() {return caller->function();}

#define CATCH_IN_LAMBDA_MEMBER(caller, function) [this](){return caller->function();}

namespace BinaryFileType {
    enum FileType {
        BENCHMARK_LEGACY_ANGLE,
        BENCHMARK_LEGACY_DISTANCE,
        BENCHMARK_LEGACY_DISTANCE_ANGLE,
        BENCHMARK_ANGLE_V_0_1,
        BENCHMARK_DISTANCE_V_0_1,
        BENCHMARK_DISTANCE_ANGLE_V_0_1,
        Z_N_LEGACY_ANGLE,
        Z_N_LEGACY_DISTANCE,
        Z_N_LEGACY_ANGLE_SPEED,
        Z_N_LEGACY_DISTANCE_SPEED,
        BENCHMARK_LEGACY_CURVE,
        BENCHMARK_CURVE_V_0_1,
        BENCHMARK_ANGLE_V_0_2,
        BENCHMARK_DISTANCE_V_0_2,
        UNIVERSAL_BENCHMARK_V0_1
    };

}

extern void printFreeRAM1();

#endif //PAMITEENSY_CONFIG_H
