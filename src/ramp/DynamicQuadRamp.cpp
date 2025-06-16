//
// Created by fogoz on 27/04/2025.
//

#include "ramp/DynamicQuadRamp.h"

#include "robot/PAMIRobot.h"


DynamicQuadRamp::DynamicQuadRamp(std::shared_ptr<BaseRobot> robot, RampData ramp, std::function<double()> distanceToPoint, std::function<double()> curvature) : robot(robot), ramp(ramp), distanceToPoint(distanceToPoint), curvature(curvature) {

}

void DynamicQuadRamp::start(double initialSpeed) {
    this->currentSpeed = initialSpeed;
}

double DynamicQuadRamp::computeDelta() {
    double v_curve = ramp.maxSpeed;
    double d_rem = distanceToPoint();
    double dt = robot->getDT();
    double curvature = std::abs(this->curvature());
    int direction = (d_rem >= 0) ? 1 : -1;
    double abs_d_rem = std::abs(d_rem);

    if (curvature > 1e-6) {
        v_curve = std::sqrt(ramp.max_lateral_accel / curvature);
        v_curve = std::clamp(v_curve, 0.0, ramp.maxSpeed);
    }

    // 2. Braking-based limit
    double v_brake = std::sqrt(2.0 * abs_d_rem * ramp.dec);

    // 3. Final target speed (positive, then apply direction)
    double v_target = std::min({v_curve, v_brake, ramp.maxSpeed});
    v_target *= direction;

    // 4. Accelerate/decelerate toward target speed
    double delta_v = v_target - currentSpeed;
    double max_delta = ((delta_v > 0) ? ramp.acc : ramp.dec) * dt;
    delta_v = std::clamp(delta_v, -std::abs(max_delta), std::abs(max_delta));

    currentSpeed += delta_v;
    return currentSpeed * dt;
}

double DynamicQuadRamp::getCurrentSpeed() {
    return currentSpeed;
}

void DynamicQuadRamp::stop() {
    this->currentSpeed = 0.0f;
}
