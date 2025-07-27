//
// Created by fogoz on 02/06/2025.
//
#include "target/BaseTarget.h"
#include "robot/BaseRobot.h"

void BaseTarget::process() {
    //TODO handle those case
    if (!robot->isDoneAngular() && robot->isDoneDistance()) {
        //Rotation seule
        Angle current_angle = robot->getRotationalPosition();
        //double current_distance = robot->getTranslationalPosition();
        if (abs((current_angle - previous_angle).toDegrees()) < Angle::fromDegrees(0.001).toDegrees()) {
            ticks++;
            if (ticks > 200) {
                done = true;
            }
        }else {
            ticks = 0;
        }
        previous_angle = current_angle;

    }else if (!robot->isDoneAngular() && !robot->isDoneDistance()) {
        //Bouge + angle
    }else if (!robot->isDoneDistance()) {
        //Bouge seul
        double current_distance = robot->getTranslationalPosition();
        if (abs(current_distance - previous_distance) < 0.01) {
            ticks++;
            if (ticks > 200) {
                done = true;
            }
        }else {
            ticks = 0;
        }
        previous_distance = current_distance;
    }
}
