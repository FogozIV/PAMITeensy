#pragma once

#include "ContinuousCurveTarget.h"
#include "robot/BaseRobot.h"

template<typename Ramp>
ContinuousCurveTarget<Ramp>::ContinuousCurveTarget(const std::shared_ptr<BaseRobot> &robot, std::shared_ptr<BaseCurve> curve,
                               RampData ramp, double ahead_distance): BaseTarget(robot), rampData(ramp), curve(curve), ahead_distance(ahead_distance) {
    this->t = 0.0f;
}

template<typename Ramp>
bool ContinuousCurveTarget<Ramp>::is_done() {
    return done;
}

template<typename Ramp>
void ContinuousCurveTarget<Ramp>::init() {
    ramp = std::make_shared<Ramp>(robot, rampData, [this]() {
        if (this->curve->isBackward()) {
            return -curve->getLength(this->t, this->curve->getMaxValue());
        }
        return curve->getLength(this->t, this->curve->getMaxValue());
    });
    streamSplitter.printf("Translational ramp speed: %f\r\n", robot->getTranslationalRampSpeed());
    this->startingCurvilinearDistance = this->robot->getTranslationalPosition();
    ramp->start(robot->getTranslationalRampSpeed());
    robot->setDoneAngular(false);
    robot->setDoneDistance(false);
    this->t = curve->getValueForLength(curve->getMinValue(), ahead_distance, 0.01);
    this->target_pos = curve->getPosition(this->t);
}

template<typename Ramp>
void ContinuousCurveTarget<Ramp>::on_done() {
    robot->setDoneAngular(true);
    robot->setDoneDistance(true);
}

template<typename Ramp>
void ContinuousCurveTarget<Ramp>::process() {
    this->t = curve->getValueForLength(curve->getMinValue(), (curve->isBackward() ? -1 : 1) * (this->robot->getTranslationalPosition() - this->startingCurvilinearDistance) + ahead_distance, 0.01);
    this->target_pos = curve->getPosition(t);
    double increment = ramp->computeDelta();
    robot->setTranslationalTarget(robot->getTranslationalTarget() + increment);
    robot->setTranslationalRampSpeed(ramp->getCurrentSpeed());
    if (this->curve->isBackward()) {
        robot->setRotationalTarget(robot->getRotationalPosition().fromUnwrapped((target_pos - robot->getCurrentPosition()).getVectorAngle()) + AngleConstants::HALF_TURN);
    }else {
        robot->setRotationalTarget(robot->getRotationalPosition().fromUnwrapped((target_pos-robot->getCurrentPosition()).getVectorAngle()));
    }
    if ((this->target_pos - robot->getCurrentPosition()).getDistance() < 10) {
        robot->setDoneAngular(true);
    }else {
        robot->setDoneAngular(false);
    }
    if(increment == 0.0f && rampData.endSpeed != 0.0f) {
        done = true;
    }else if (increment == 0.0f && abs(robot->getTranslationalTarget() - robot->getTranslationalPosition()) < robot->getTolerances()->curvilinear_tolerance) {
        tick++;
        if (tick > robot->getTolerances()->ticks_in_curvilinear_tolerance) {
            done = true;
        }
    }
    BaseTarget::process();
}

template<typename Ramp>
void ContinuousCurveTarget<Ramp>::reInitAfterStop() {
    ramp->stop();
}

template<typename Ramp>
Position ContinuousCurveTarget<Ramp>::getTargetPosition() {
    return target_pos;
}