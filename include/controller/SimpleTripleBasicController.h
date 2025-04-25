//
// Created by fogoz on 24/04/2025.
//

#ifndef PAMITEENSY_SIMPLETRIPLEBASICCONTROLLER_H
#define PAMITEENSY_SIMPLETRIPLEBASICCONTROLLER_H

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
    TripleBasicParameters params;
public:

    SimpleTripleBasicController(const std::shared_ptr<BasicController> &distanceController,
                                const std::shared_ptr<BasicController> &distanceAngleController,
                                const std::shared_ptr<BasicController> &angleController,
                                const std::shared_ptr<BaseRobot> &robot, const TripleBasicParameters &params);

    void compute() override;

    void reset(bool correct_error=false) override;
};


#endif //PAMITEENSY_SIMPLETRIPLEBASICCONTROLLER_H
