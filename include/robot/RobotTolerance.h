//
// Created by fogoz on 12/06/2025.
//

#ifndef ROBOTTOLERANCE_H
#define ROBOTTOLERANCE_H

#include "ArduinoJson.h"

struct RobotTolerance {
    double curvilinear_tolerance = 2;
    uint16_t ticks_in_curvilinear_tolerance = 20;
    double angular_tolerance = 1;
    uint16_t ticks_in_angular_tolerance = 20;
    double distance_to_point = 5;
    uint16_t ticks_in_distance_to_point = 20;
};

ARDUINOJSON_BEGIN_PUBLIC_NAMESPACE
template <>
struct Converter<RobotTolerance> {
    static void toJson(const RobotTolerance& src, JsonVariant dst) {
        dst["curvilinear_tolerance"] = src.curvilinear_tolerance;
        dst["ticks_in_curvilinear_tolerance"] = src.ticks_in_curvilinear_tolerance;
        dst["angular_tolerance"] = src.angular_tolerance;
        dst["ticks_in_angular_tolerance"] = src.ticks_in_angular_tolerance;
        dst["distance_to_point"] = src.distance_to_point;
        dst["ticks_in_distance_to_point"] = src.ticks_in_distance_to_point;
    }
    static RobotTolerance fromJson(JsonVariantConst src) {
        RobotTolerance r{src["curvilinear_tolerance"].as<double>(), src["ticks_in_curvilinear_tolerance"].as<uint16_t>(),src["angular_tolerance"].as<double>(),src["ticks_in_angular_tolerance"].as<uint16_t>()};
        if (src["distance_to_point"].is<double>()) {
            r.distance_to_point = src["distance_to_point"].as<double>();
        }
        if (src["ticks_in_distance_to_point"].is<uint16_t>()) {
            r.ticks_in_distance_to_point = src["ticks_in_distance_to_point"].as<uint16_t>();
        }
        return r;
    }
    static bool checkJson(JsonVariantConst src) {
        return src["curvilinear_tolerance"].is<double>() && src["ticks_in_curvilinear_tolerance"].is<uint16_t>() && src["angular_tolerance"].is<double>() && src["ticks_in_angular_tolerance"].is<uint16_t>();
    }
};
ARDUINOJSON_END_PUBLIC_NAMESPACE

#endif //ROBOTTOLERANCE_H
