//
// Created by fogoz on 08/05/2025.
//

#include "../../include/target/AngleTarget.h"

#include "ramp/BasicQuadRamp.h"
#include "robot/BaseRobot.h"

bool AngleTarget::is_done() {
    return done;
}

void AngleTarget::init() {
    robot->setTranslationalPosition(robot->getTranslationalPosition());
    ramp = std::make_shared<BasicQuadRamp>(robot, ramp_data, [this]() {
        return (target_angle - robot->getTranslationalPosition()).toDegrees();
    });
    ramp->start(robot->getTranslationalRampSpeed());
    robot->setDoneDistance(false);
    robot->setDoneAngular(true);
}

void AngleTarget::process() {
    double target = ramp->computeDelta();
    robot->setRotationalRampSpeed(Angle::fromDegrees(ramp->getCurrentSpeed()));
    robot->setRotationalTarget(robot->getRotationalPosition() + Angle::fromDegrees(target));
    if ((target_angle - robot->getTranslationalPosition()).toDegrees() < 2) {
        robot->setDoneDistance(true);
        done = true;
    }
}

void AngleTarget::reInitAfterStop() {
    ramp->stop();
}
