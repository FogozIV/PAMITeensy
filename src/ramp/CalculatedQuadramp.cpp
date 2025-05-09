//
// Created by fogoz on 08/05/2025.
//

#include "../../include/ramp/CalculatedQuadramp.h"

#include "utils/BufferFilePrint.h"

CalculatedQuadramp::CalculatedQuadramp(std::shared_ptr<BaseRobot> robot, RampData data,
                                       std::function<double()> fct): robot(robot), data(data), distanceToPoint(fct) {
}

double CalculatedQuadramp::computeAtTime(double t) {
    if (t < 0) {
        return 0;
    }
    double t1 = abs(calculatedData.acc_time);
    double t2 = t1 + abs(calculatedData.ste_time);
    double t3 = t2 + abs(calculatedData.dec_time);
    if (t < t1) {
        current_speed = calculatedData.initial_speed  + copysign(1, calculatedData.acc_time) * data.acc * t;
        double result= calculatedData.initial_speed *t + copysign(1, calculatedData.acc_time) * data.acc * pow(t,2) / 2.0f;
        bufferPrinter->printf("quadramp_1= %f; %f; %f; %f\r\n", result, current_speed, t, t);
        return result;
    }
    if (t < t2) {
        double local_t = t - t1;
        current_speed = calculatedData.ste_speed;
        double result= calculatedData.acc_distance + calculatedData.ste_speed * local_t;
        bufferPrinter->printf("quadramp_2= %f; %f; %f; %f\r\n", result, current_speed, local_t, t);
        return result;
    }
    if (t < t3) {
        double local_t = t - t2;
        current_speed = calculatedData.ste_speed -copysign(1, calculatedData.dec_time) * data.acc * local_t;
        double result = calculatedData.acc_distance + calculatedData.ste_speed * calculatedData.ste_time + calculatedData.ste_speed * local_t -copysign(1, calculatedData.dec_time) * data.acc * pow(local_t,2) / 2.0f;
        bufferPrinter->printf("quadramp_3= %f; %f; %f; %f\r\n", result, current_speed, local_t, t);
        return result;
    }
    double result = calculatedData.acc_distance + calculatedData.ste_speed * calculatedData.ste_time + calculatedData.dec_distance;
    current_speed = calculatedData.end_speed;
    bufferPrinter->printf("quadramp_4= %f; %f; %f; %f\r\n", result, current_speed, t, t);
    return result;
}

void CalculatedQuadramp::start(double initialSpeed) {
    double distance = distanceToPoint();
    calculatedData.inversed = distance < 0;
    float initSpeed = initialSpeed;
    float endSpeed = data.endSpeed;
    float maxSpeed = calculatedData.inversed ? -data.maxSpeed : data.maxSpeed;

    calculatedData.initial_distance = distance;

    calculatedData.initial_speed = initSpeed;
    current_speed = initSpeed;
    calculatedData.end_speed = endSpeed;

    calculatedData.acc_time = (maxSpeed - initSpeed) / data.acc;
    calculatedData.dec_time = (maxSpeed - endSpeed) / data.acc;
    calculatedData.acc_distance = calculatedData.initial_speed * abs(calculatedData.acc_time) + copysign(1, calculatedData.acc_time) * data.acc * pow(
                                      calculatedData.acc_time, 2) / 2.0f;
    calculatedData.dec_distance = maxSpeed * abs(calculatedData.dec_time) - copysign(1, calculatedData.dec_time) *data.acc * pow(calculatedData.dec_time, 2) /
                                  2.0f;

    if (abs(calculatedData.acc_distance + calculatedData.dec_distance) <= abs(distance)) {
        //Found using octave and basic movement equations
        calculatedData.ste_time = (2 * data.acc * abs(distance) + pow(endSpeed, 2) + pow(initSpeed, 2) - 2 *
                                   pow(maxSpeed, 2)) / (2 * data.acc * maxSpeed);
        calculatedData.ste_speed = maxSpeed;
    } else {
        calculatedData.ste_time = 0.0f;
        double inside_sqrt = 4 * data.acc * abs(distance) + 2 * pow(endSpeed, 2) + 2 * pow(initSpeed, 2);
        calculatedData.ste_speed = copysign(1, maxSpeed) *sqrt(max(inside_sqrt, 0)) / 2;
        calculatedData.dec_time = (-endSpeed + calculatedData.ste_speed) / data.acc;
        calculatedData.acc_time = (-initSpeed + calculatedData.ste_speed) / data.acc;
    }
    calculatedData.acc_distance = calculatedData.initial_speed * abs(calculatedData.acc_time) + copysign(1, calculatedData.acc_time) *data.acc * pow(
                                      calculatedData.acc_time, 2) / 2.0f;
    calculatedData.dec_distance = calculatedData.ste_speed * abs(calculatedData.dec_time) - copysign(1, calculatedData.dec_time) *data.acc * pow(calculatedData.dec_time, 2) /2.0f;

    bufferPrinter->printf("quadramp= %f; %f; %f; %f; %f; %f; %f\r\n", calculatedData.acc_time, calculatedData.ste_time, calculatedData.dec_time, calculatedData.acc_distance, calculatedData.ste_speed, calculatedData.dec_distance, distance);

}

double CalculatedQuadramp::computeDelta() {
    t+=robot->getDT();
    double current = computeAtTime(t);
    //On swap
    std::swap(previous_value, current);
    //current - previous_value but since we swapped it's the opposite
    return previous_value - current;

}

double CalculatedQuadramp::getCurrentSpeed() {
    return current_speed;
}

void CalculatedQuadramp::stop() {
    start(0);
}

CalculatedQuadramp::~CalculatedQuadramp() {
}
