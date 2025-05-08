//
// Created by fogoz on 07/05/2025.
//

#include "../../include/target/PositionTarget.h"

#include "ramp/BasicQuadRamp.h"


PositionTarget::PositionTarget(std::shared_ptr<BaseRobot> baseRobot, Position pos, RampData rampData): BaseTarget(baseRobot),pos(pos), ramp_data(rampData) {
}

bool PositionTarget::is_done() {
    return done;
}

void PositionTarget::init() {
    ramp = std::make_shared<BasicQuadRamp>(robot, ramp_data, [this]() {
        Angle angle = ((pos-robot->getCurrentPosition()).getVectorAngle() - robot->getRotationalPosition()).warpAngle();
        if (angle > -90_deg && angle < 90_deg) {
            return (robot->getCurrentPosition() - pos).getDistance();
        }
        return -(robot->getCurrentPosition() - pos).getDistance();
    });
    ramp->start(robot->getTranslationalRampSpeed());
    robot->setDoneAngular(false);
    robot->setDoneDistance(false);
    robot->setRotationalPosition(robot->getCurrentPosition().getAngle());
}

void PositionTarget::process() {
    double distance_update = robot->getCurrentPosition().getDistance();
    robot->setTranslationalRampSpeed(ramp->getCurrentSpeed());
    robot->setTranslationalTarget(robot->getTranslationalTarget() + distance_update);
    robot->setRotationalTarget((pos-robot->getCurrentPosition()).getVectorAngle());
    double distance = (pos-robot->getCurrentPosition()).getDistance();
    if (distance < 10) {
        robot->setDoneAngular(true);
    }else {
        robot->setDoneAngular(false);
    }
    if (distance < 5) {
        if (ramp_data.endSpeed == 0) {
            robot->setTranslationalTarget(robot->getTranslationalPosition());
        }
        done = true;
    }
}

void PositionTarget::on_done() {
    if (ramp_data.endSpeed == 0)
        robot->setTranslationalTarget(robot->getTranslationalPosition());
    robot->setRotationalTarget(robot->getRotationalPosition());
    robot->setDoneDistance(true);
    robot->setDoneAngular(true);
}

void PositionTarget::reInitAfterStop() {
    ramp->stop();
}




