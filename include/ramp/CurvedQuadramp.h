//
// Created by fogoz on 08/06/2025.
//

#ifndef CURVEDQUADRAMP_H
#define CURVEDQUADRAMP_H
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include "ramp/Ramp.h"
#include "robot/BaseRobot.h"


class CurvedQuadramp {
protected:
    std::shared_ptr<BaseRobot> robot;
    RampData rampData;
    bool angle;
public:

    CurvedQuadramp(std::shared_ptr<BaseRobot> robot, RampData ramp, bool angle=false): robot(robot), rampData(ramp), angle(angle) {
    }

    void reset(double pos, double speed = 0.0) {
        this->speed = speed;
        this->position = pos;
    }

    // Main function
    double update() {
        double target_pos;
        double current_pos;
        double current_speed;
        double curvature = 0;
        if (this->angle) {
            target_pos = this->robot->getRotationalPosition().toDegrees();
            current_pos = this->robot->getRotationalPosition().toDegrees();
            current_speed = this->robot->getRotationalEstimatedSpeed().toDegrees();
        }else {
            target_pos = this->robot->getTranslationalTarget();
            current_pos = this->robot->getTranslationalPosition();
            current_speed = this->robot->getTranslationalEstimatedSpeed();
        }


        double dt = this->robot->getDT();
        double dx = target_pos - current_pos;
        double sign_dx = (dx >= 0) ? 1.0 : -1.0;

        // 1. Compute curvature-based speed limit
        double v_curve_limit = std::numeric_limits<double>::infinity();
        if (!this->angle) {
            if (std::abs(curvature) > 1e-6) {
                v_curve_limit = std::sqrt(std::abs(lat_acc_limit / curvature));
            }
        }

        // 2. Compute safe stop speed
        double v_stop_limit = std::sqrt(2.0 * dec_limit * std::abs(dx));
        v_stop_limit *= sign_dx;

        // 3. Clamp to dynamic max speed (from curvature)
        double v_target = std::clamp(v_stop_limit, -v_curve_limit, v_curve_limit);

        // 4. Acceleration limit (asymmetric)
        double max_delta_v = (v_target > current_speed ? acc_limit : dec_limit) * dt;
        double v_next = std::clamp(v_target, current_speed - max_delta_v, current_speed + max_delta_v);

        // 5. Optional: Snap to stop near target
        if (std::abs(dx) < snap_position_threshold && std::abs(current_speed) < snap_speed_threshold) {
            v_next = 0.0;
        }

        speed = v_next;
        return speed;
    }

    void setSnapThresholds(double pos_eps, double speed_eps) {
        snap_position_threshold = pos_eps;
        snap_speed_threshold = speed_eps;
    }

private:
    double acc_limit;
    double dec_limit;
    double lat_acc_limit;
    double speed = 0.0;
    double position = 0.0;

    // Optional snap-to-target thresholds
    double snap_position_threshold = 0.001; // 1 mm
    double snap_speed_threshold = 0.01;     // 1 cm/s
};



#endif //CURVEDQUADRAMP_H
