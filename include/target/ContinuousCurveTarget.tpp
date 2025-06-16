#pragma once

#include "ContinuousCurveTarget.h"
#include "ramp/DynamicQuadRamp.h"

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
    this->final_pos = curve->getLastPosition();
}

template<>
inline void ContinuousCurveTarget<DynamicQuadRamp>::init() {
    ramp = std::make_shared<DynamicQuadRamp>(robot, rampData, [this]() {
        double current_distance = curve->getLength(this->t, this->curve->getMaxValue());
        double point_distance = (this->getTargetPosition() - robot->getCurrentPosition()).getDistance();
        if (point_distance > abs(current_distance)) {
            return robot->getCurrentPosition().isBehind(this->getTargetPosition()) ? - point_distance : point_distance;
        }
        if (this->curve->isBackward()) {
            return -current_distance;
        }
        return current_distance;
    }, [this]() {
        return curve->getPosition(this->t).getCurvature();
    });
    this->startingCurvilinearDistance = this->robot->getTranslationalPosition();
    ramp->start(robot->getTranslationalRampSpeed());
    robot->setDoneAngular(false);
    robot->setDoneDistance(false);
    this->t = curve->getValueForLength(curve->getMinValue(), ahead_distance, 0.01);
    this->target_pos = curve->getPosition(this->t);
    this->final_pos = curve->getLastPosition();
}

template<typename Ramp>
void ContinuousCurveTarget<Ramp>::on_done() {
    robot->setDoneAngular(true);
    robot->setDoneDistance(true);
}

template<typename Ramp>
void ContinuousCurveTarget<Ramp>::process() {
    //this->t = this->curve->findNearest(this->robot->getCurrentPosition(), this->t, 0.01, 20, this->t - 0.2 * (this->curve->getMaxValue() - this->curve->getMinValue()), this->t + 0.2 * (this->curve->getMaxValue() - this->curve->getMinValue()));
    //this->t = curve->getValueForLength(this->t, ahead_distance, 0.01);
    this->t = curve->getValueForLength(curve->getMinValue(), (this->curve->isBackward() ? -1 : 1) * (this->robot->getTranslationalPosition() - this->startingCurvilinearDistance)  +ahead_distance, 0.01);
    this->target_pos = curve->getPosition(t);
    double increment = ramp->computeDelta();
    robot->setTranslationalTarget(robot->getTranslationalTarget() + increment);
    robot->setTranslationalRampSpeed(ramp->getCurrentSpeed());
    if (this->curve->isBackward()) {
        robot->setRotationalTarget(robot->getRotationalPosition().fromUnwrapped((target_pos - robot->getCurrentPosition()).getVectorAngle()) + AngleConstants::HALF_TURN);
    }else {
        robot->setRotationalTarget(robot->getRotationalPosition().fromUnwrapped((target_pos-robot->getCurrentPosition()).getVectorAngle()));
    }
    if ((this->final_pos - robot->getCurrentPosition()).getDistance() < 10) {
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
    }else if ((final_pos - robot->getCurrentPosition()).getDistance() < robot->getTolerances()->distance_to_point) {
        tick++;
        if (tick > robot->getTolerances()->ticks_in_distance_to_point) {
            done = true;
        }
    }else {
        tick = 0;
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