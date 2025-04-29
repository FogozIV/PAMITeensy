//
// Created by fogoz on 27/04/2025.
//

#ifndef RAMP_H
#define RAMP_H



class Ramp {
public:
    virtual ~Ramp() = default;

    virtual void start(double initialSpeed) = 0;

    virtual double computeDelta() = 0;
};



#endif //RAMP_H
