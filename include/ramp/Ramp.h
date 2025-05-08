//
// Created by fogoz on 27/04/2025.
//

#ifndef RAMP_H
#define RAMP_H

struct RampData{
    double acc;
    double maxSpeed;
    double endSpeed = 0;
public:
    RampData(double acc, double maxSpeed, double endSpeed=0) : acc(acc), maxSpeed(maxSpeed), endSpeed(endSpeed){};
};

class Ramp {
public:
    virtual ~Ramp() = default;

    virtual void start(double initialSpeed) = 0;

    virtual double computeDelta() = 0;

    virtual double getCurrentSpeed() = 0;

    virtual void stop();
};



#endif //RAMP_H
