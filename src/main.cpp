

#include <Arduino.h>
#include "robot/PAMIRobot.h"

#include "TeensyThreads.h"
std::shared_ptr<PAMIRobot> robot;

void setup() {
    Serial.begin(1000000);
    robot = std::make_shared<PAMIRobot>();
    robot->init(robot);

    robot->save();
}

void loop() {

}