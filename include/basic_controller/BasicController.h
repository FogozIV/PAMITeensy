//
// Created by fogoz on 24/04/2025.
//

#ifndef PAMITEENSY_BASICCONTROLLER_H
#define PAMITEENSY_BASICCONTROLLER_H


class BasicController {
public:
    virtual double evaluate(double error);
    virtual void reset(double error=0.0f);
};


#endif //PAMITEENSY_BASICCONTROLLER_H
