//
// Created by fogoz on 24/04/2025.
//

#ifndef PAMITEENSY_BASICCONTROLLER_H
#define PAMITEENSY_BASICCONTROLLER_H


class BasicController {
public:
    virtual double evaluate(double error) = 0;
    virtual void reset(double error) = 0;
    void reset(){
        reset(0.0f);
    }
    virtual ~BasicController() = default;
};


#endif //PAMITEENSY_BASICCONTROLLER_H
