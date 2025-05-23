//
// Created by fogoz on 10/05/2025.
//
#pragma once

#include "CurveTarget.h"
#include "robot/BaseRobot.h"

template<typename Ramp>
CurveTarget<Ramp>::CurveTarget(const std::shared_ptr<BaseRobot> &robot, std::shared_ptr<BaseCurve> curve,
    RampData ramp, double step): BaseTarget(robot), rampData(ramp), curve(curve), step(step) {
    t = curve->getMinValue();
}

template<typename Ramp>
bool CurveTarget<Ramp>::is_done() {
    return done;
}

template<typename Ramp>
void CurveTarget<Ramp>::init() {
    ramp = std::make_shared<Ramp>(robot, rampData, [this]() {
        if (this->curve->isBackward()) {
            return -curve->getLength(this->t, this->curve->getMaxValue());
        }
        return curve->getLength(this->t, this->curve->getMaxValue());
    });
    ramp->init();
    robot->setDoneAngular(false);
    robot->setDoneDistance(false);
    robot->setTranslationalTarget(robot->getTranslationalPosition());
    robot->setRotationalTarget(robot->getRotationalPosition());
    this->t = curve->getValueForLength(this->t, step, 0.01);
    this->target_pos = curve->getPosition(this->t);
}

template<typename Ramp>
void CurveTarget<Ramp>::on_done() {
    robot->setDoneAngular(true);
    robot->setDoneDistance(true);
    if (rampData.endSpeed) {
        robot->setTranslationalTarget(robot->getTranslationalTarget());
    }
}

template<typename Ramp>
void CurveTarget<Ramp>::process() {
    double distance = (target_pos - robot->getCurrentPosition()).getDistance();
    double increment = ramp->computeDelta();
    robot->setTranslationalTarget(robot->getTranslationalTarget() + increment);
    robot->setTranslationalRampSpeed(ramp->getCurrentSpeed());
    if ((target_pos - robot->getCurrentPosition()).getDistance() < step/2 && t!= curve->getMaxValue()) {
        this->t = this->curve->getValueForLength(this->t, step, 0.01);
    }
    if (this->curve->isBackward()) {
        robot->setRotationalTarget(robot->getRotationalTarget().fromUnwrapped((robot->getCurrentPosition()-target_pos).getVectorAngle()) + AngleConstants::HALF_TURN);
    }else {
        robot->setRotationalTarget(robot->getRotationalTarget().fromUnwrapped((target_pos-robot->getCurrentPosition()).getVectorAngle()));
    }
    if (t >= curve->getMaxValue() && distance < 10) {
        robot->setDoneDistance(true);
        done = true;
    }
    if (distance < 10) {
        robot->setDoneAngular(true);
    }else {
        robot->setDoneAngular(false);
    }
}

template<typename Ramp>
void CurveTarget<Ramp>::reInitAfterStop() {
    ramp->stop();
}
