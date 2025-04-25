//
// Created by fogoz on 24/04/2025.
//

#ifndef PAMITEENSY_SIMPLETRIPLEBASICCONTROLLER_H
#define PAMITEENSY_SIMPLETRIPLEBASICCONTROLLER_H

#include "ArduinoJson.h"

#include "BaseController.h"
#include "memory"
#include "basic_controller/BasicController.h"

class BaseRobot;

struct TripleBasicParameters{
    double speed_min_l = 0;
    double speed_min_r = 0;

    double maxValueDistance = 4095;
    double maxValueDistanceAngle = 4095;
    double maxValueAngular = 4095;
};


class SimpleTripleBasicController: public BaseController {
    std::shared_ptr<BasicController> distanceController;
    std::shared_ptr<BasicController> distanceAngleController;
    std::shared_ptr<BasicController> angleController;
    std::shared_ptr<BaseRobot> robot;
    std::shared_ptr<TripleBasicParameters> params;
public:

    SimpleTripleBasicController(
            const std::shared_ptr<BaseRobot> &robot,const std::shared_ptr<BasicController> &distanceController,
                                const std::shared_ptr<BasicController> &distanceAngleController,
                                const std::shared_ptr<BasicController> &angleController, const std::shared_ptr<TripleBasicParameters> &params);

    void compute() override;

    void reset(bool correct_error) override;
};

ARDUINOJSON_BEGIN_PUBLIC_NAMESPACE
template <>
struct Converter<TripleBasicParameters> {
    static void toJson(const TripleBasicParameters& src, JsonVariant dst) {
        dst["speed_min_l"] = src.speed_min_l;
        dst["speed_min_r"] = src.speed_min_r;
        dst["maxValueDistance"] = src.maxValueDistance;
        dst["maxValueDistanceAngle"] = src.maxValueDistanceAngle;
        dst["maxValueAngular"] = src.maxValueAngular;
    }

    static TripleBasicParameters fromJson(JsonVariantConst src) {
        return {src["speed_min_l"].as<double>(),src["speed_min_r"].as<double>(),src["maxValueDistance"].as<double>(),src["maxValueDistanceAngle"].as<double>(), src["maxValueAngular"].as<double>()};
    }

    static bool checkJson(JsonVariantConst src) {
        return src["speed_min_l"].is<double>() && src["speed_min_r"].is<double>() && src["maxValueDistance"].is<double>() && src["maxValueAngular"].is<double>() && src["maxValueDistanceAngle"].is<double>();
    }
};

ARDUINOJSON_END_PUBLIC_NAMESPACE

#endif //PAMITEENSY_SIMPLETRIPLEBASICCONTROLLER_H
