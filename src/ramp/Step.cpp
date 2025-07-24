//
// Created by fogoz on 23/07/2025.
//

#include <ramp/Step.h>

Step::Step(std::shared_ptr<BaseRobot> robot, RampData ramp_data, std::function<double()> distance)  {
    this->distance = distance;
}
void Step::start(double initialSpeed) {
    distance_send = distance();
    sent = false;
}

double Step::computeDelta() {
    if (!sent) {
        sent = true;
        return distance_send;
    }
    return 0;
}

double Step::getCurrentSpeed() {
    return 0.0f;
}

void Step::stop() {

}
