//
// Created by fogoz on 24/04/2025.
//

#ifndef PAMITEENSY_PAMIROBOT_H
#define PAMITEENSY_PAMIROBOT_H

#include <list>
#include "robot/BaseRobot.h"
#include "utils/PositionManager.h"
#include "basic_controller/PID.h"
#include "controller/SimpleTripleBasicController.h"
#include "chrono"
#include "ArduinoJson.h"
#include <FS.h>
#include <SD.h>

/**
 * This implementation of BaseRobot is using PID's QuadEncoderImpl and DirPWMMotor for it's implementation
 */
class PAMIRobot : public BaseRobot{
protected:
    std::list<std::shared_ptr<BaseTarget>> targets;
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::duration<double, std::ratio<1,1>>> previous_time;
    double dt = 0.005;

    std::shared_ptr<PID> pidDistance;
    std::shared_ptr<PID> pidAngle;
    std::shared_ptr<PID> pidDistanceAngle;

    std::shared_ptr<TripleBasicParameters> pidParameters;

    std::shared_ptr<BaseEncoder> leftEncoder;
    std::shared_ptr<BaseEncoder> rightEncoder;

    std::shared_ptr<PositionParameters> parameters;

public:
    double getDT() override;

    void computeTarget() override;

    void computePosition() override;

    void computeController() override;

    void addTarget(std::shared_ptr<BaseTarget> target) override;

    void compute() override;

    void init(std::shared_ptr<PAMIRobot> robot);
};


#endif //PAMITEENSY_PAMIROBOT_H
