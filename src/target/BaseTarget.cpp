//
// Created by fogoz on 02/06/2025.
//
#include "target/BaseTarget.h"
#include "robot/BaseRobot.h"

void BaseTarget::process() {
    //TODO handle those case
    if (!robot->isDoneAngular() && robot->isDoneDistance()) {
        //Rotation seule

    }else if (!robot->isDoneAngular() && !robot->isDoneDistance()) {
        //Bouge + angle
    }else if (!robot->isDoneDistance()) {
        //Bouge seul
    }
}
