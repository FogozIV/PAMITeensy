//
// Created by fogoz on 10/05/2025.
//

#ifndef CLOTHOID_H
#define CLOTHOID_H
#include "BaseCurve.h"


class ClothoidCurve : public BaseCurve {
    Position start;
    double startCurvature;
    double a;
    double length;
    std::tuple<double, Position> previousPos= {-1, {}};
    std::tuple<double, Position> previousDerivative= {-1, {}};
    void clothoidODE(double t, std::vector<double> &y, std::vector<double> &dydt) const;
public:
    ClothoidCurve(Position start, double startCurvature, double a, double length);

    Position getPosition(double value, double h) override;


    Position getDerivative(double value) override;

    double getLength(double ti, double t_end, double h) override;

    double getValueForLength(double ti, double length, double h) override;
};



#endif //CLOTHOID_H
