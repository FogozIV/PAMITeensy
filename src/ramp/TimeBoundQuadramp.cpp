//
// Created by fogoz on 08/06/2025.
//

#include <ramp/TimeBoundQuadramp.h>

TimeBoundQuadramp::TimeBoundQuadramp(std::shared_ptr<BaseRobot> robot, RampData data, std::function<double()> fct) : data(data), robot(robot), fct(fct) {
}

void TimeBoundQuadramp::start(double initialSpeed) {
    double initSpeed = initialSpeed;
    double endSpeed = data.endSpeed;
    double T = this->fct();
    double a = data.acc;
    double d = data.dec;

    // Try to build trapezoid, check if a trapezoid or triangle fits
    double t1 = (data.maxSpeed - initSpeed) / a;
    double t3 = (data.maxSpeed - endSpeed) / d;

    double t2 = T - t1 - t3;

    if (t2 >= 0) {
        // Trapezoidal profile
        calculatedData.acc_time = t1;
        calculatedData.dec_time = t3;
        calculatedData.ste_time = t2;
        calculatedData.ste_speed = data.maxSpeed;
    } else {
        // Triangular profile — can't reach max speed
        calculatedData.ste_time = 0;
        double v_peak = sqrt((2 * a * d * T * (initSpeed * d + endSpeed * a) + (d * initSpeed * initSpeed + a * endSpeed * endSpeed) * (a + d)) /
                             (a + d));
        v_peak = std::min(v_peak, data.maxSpeed);  // Clamp if needed

        calculatedData.acc_time = (v_peak - initSpeed) / a;
        calculatedData.dec_time = (v_peak - endSpeed) / d;
        calculatedData.ste_speed = v_peak;
    }

    calculatedData.initial_speed = initSpeed;
    calculatedData.end_speed = endSpeed;
    current_speed = initSpeed;
    calculatedData.acc_distance = calculatedData.initial_speed * abs(calculatedData.acc_time)
                            + copysign(1, calculatedData.acc_time) * data.acc * pow(calculatedData.acc_time, 2) / 2.0f;

    calculatedData.dec_distance = calculatedData.ste_speed * abs(calculatedData.dec_time)
                                - copysign(1, calculatedData.dec_time) * data.dec * pow(calculatedData.dec_time, 2) / 2.0f;

}

double TimeBoundQuadramp::computeAtTime(double t) {
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
        current_speed = calculatedData.ste_speed -copysign(1, calculatedData.dec_time) * data.dec * local_t;
        double result = calculatedData.acc_distance + calculatedData.ste_speed * abs(calculatedData.ste_time) + calculatedData.ste_speed * local_t -copysign(1, calculatedData.dec_time) * data.dec * pow(local_t,2) / 2.0f;
        bufferPrinter->printf("quadramp_3= %f; %f; %f; %f\r\n", result, current_speed, local_t, t);
        return result;
    }
    double result = calculatedData.acc_distance + calculatedData.ste_speed * abs(calculatedData.ste_time) + calculatedData.dec_distance;
    current_speed = calculatedData.end_speed;
    bufferPrinter->printf("quadramp_4= %f; %f; %f; %f\r\n", result, current_speed, t, t);
    return result;

}

double TimeBoundQuadramp::computeDelta() {
    t+=robot->getDT();
    double current = computeAtTime(t);
    //On swap
    std::swap(previous_value, current);
    //current - previous_value but since we swapped it's the opposite
    return previous_value - current;
}

double TimeBoundQuadramp::getCurrentSpeed() {
    return current_speed;
}

void TimeBoundQuadramp::stop() {
    start(0);
}
