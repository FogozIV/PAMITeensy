//
// Created by fogoz on 10/05/2025.
//

#ifndef BEZIERCURVE_H
#define BEZIERCURVE_H
#include "BaseCurve.h"


class BezierCurve : public BaseCurve{
    Position pos1;
    Position pos2;
    Position pos3;
    Position pos4;
public:
    BezierCurve(Position pos1, Position pos2, Position pos3, Position pos4);

    Position getPosition(double value, double h) override;

    Position getDerivative(double value) override;
};


#endif //BEZIERCURVE_H
