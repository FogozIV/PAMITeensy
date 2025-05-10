//
// Created by fogoz on 10/05/2025.
//

#include "../../include/curves/BaseCurve.h"

BaseCurve::BaseCurve(double minValue, double maxValue) : minValue(minValue), maxValue(maxValue){
}

double BaseCurve::getMinValue() const {
    return minValue;
}

double BaseCurve::getMaxValue() const {
    return maxValue;
}

double BaseCurve::getLength(const double h) {
    return getLength(minValue, maxValue, h);
}

double BaseCurve::getLength(double ti, double t_end, double h) {
    if (isnan(ti)) {
        ti = minValue;
    }
    std::vector<double> y0 = {0.0};
    auto lambda = [this](double t, std::vector<double> &y, std::vector<double> &dydt) {
        dydt[0] = this->getDerivative(t).getDistance();
    };
    runge_kutta_4(ti, y0, t_end, h, lambda);
    return y0[0];
}

double BaseCurve::getValueForLength(double ti, double length, double h) {
    if (isnan(ti))
        ti = minValue;
    std::vector<double> y0 = {0.0};
    return runge_kutta_4_maximized(ti, y0, maxValue, h, [this](double t, std::vector<double> &y, std::vector<double> &dydt) {
        dydt[0] = this->getDerivative(t).getDistance();
    }, {length});
}

double BaseCurve::findNearest(Position pos, double h) {
    double dist = (getPosition(maxValue) - pos).getDistance();
    double index = maxValue;
    for (double i = minValue; i < maxValue; i += h) {
        double d = (getPosition(i) - pos).getDistance();
        if (d < dist) {
            dist = d;
            index = i;
        }
    }
    return index;
}

bool BaseCurve::isBackward() {
    return backward;
}
