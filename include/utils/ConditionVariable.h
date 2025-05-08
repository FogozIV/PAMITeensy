//
// Created by fogoz on 08/05/2025.
//

#ifndef CONDITIONVARIABLE_H
#define CONDITIONVARIABLE_H
#include <functional>

class ConditionalVariable{
    double value;
    std::function<void(double)> applyFunction;
public:

    ConditionalVariable(double bandwidth=0.0f, std::function<void(double)> fct=nullptr): value(bandwidth), applyFunction(fct) {}


    ConditionalVariable& operator=(double value) {
        this->value = value;
        if(applyFunction != nullptr){
            applyFunction(value);
        }
        return *this;
    }

    ConditionalVariable& operator=(ConditionalVariable bandwidth) {
        this->value = bandwidth.value;
        if (this->applyFunction == nullptr) {
            this->applyFunction = bandwidth.applyFunction;
        }
        if(applyFunction != nullptr){
            applyFunction(bandwidth.value);
        }
        return *this;
    }
    operator double() const {
        return value;
    }

};

#endif //CONDITIONVARIABLE_H
