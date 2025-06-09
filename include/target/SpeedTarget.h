//
// Created by fogoz on 08/06/2025.
//

#ifndef SPEEDTARGET_H
#define SPEEDTARGET_H
#include "BaseTarget.h"
#include "ramp/Ramp.h"
#include "robot/BaseRobot.h"

#define MAKE_SPEED_TARGET(rampdata, maxTime, distanceMode, ramp_type) \
std::make_shared<SpeedTarget<ramp_type>>(robot, rampdata, maxTime, distanceMode)

#define DEFAULT_MAKE_SPEED_TARGET(rampdata, maxTime, distanceMode) MAKE_SPEED_TARGET(rampdata, maxTime, distanceMode, TimeBoundQuadramp)

#define COMPLETE_SPEED_TARGET(rampdata, maxTime, distanceMode) robot->addTarget(DEFAULT_MAKE_SPEED_TARGET(rampdata, maxTime, distanceMode))

template<typename Ramp>
class SpeedTarget : public BaseTarget {

protected:
    RampData ramp_data;
    double maxTime;
    bool distanceMode;
    std::shared_ptr<Ramp> ramp;
public:
    SpeedTarget(const std::shared_ptr<BaseRobot> &robot, RampData ramp_data, double maxTime, bool distanceMode)
        : BaseTarget(robot), ramp_data(ramp_data){
        this->maxTime = maxTime;
        this->distanceMode = distanceMode;
    }

    void init() override {
        BaseTarget::init();
        ramp = std::make_shared<Ramp>(robot, ramp_data, [this](){return this->maxTime;});
        if (this->distanceMode) {
            ramp->start(robot->getTranslationalEstimatedSpeed());
        }else {
            ramp->start(robot->getRotationalEstimatedSpeed().toDegrees());
        }
    }

    void on_done() override {
        BaseTarget::on_done();

    }

    void process() override {
        double update = ramp->computeDelta();
        if (this->distanceMode) {
            this->robot->setTranslationalRampSpeed(ramp->getCurrentSpeed());
        }else {
            this->robot->setRotationalRampSpeed(Angle::fromDegrees(ramp->getCurrentSpeed()));
        }
        if (update == 0.0f) {
            done = true;
        }
        BaseTarget::process();
    }
};
#endif //SPEEDTARGET_H
