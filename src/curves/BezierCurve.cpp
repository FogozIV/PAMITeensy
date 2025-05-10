//
// Created by fogoz on 10/05/2025.
//

#include "curves/BezierCurve.h"

BezierCurve::BezierCurve(Position pos1, Position pos2, Position pos3, Position pos4): BaseCurve(0, 1.0) {
    this->pos1 = pos1;
    this->pos2 = pos2;
    this->pos3 = pos3;
    this->pos4 = pos4;
}

Position BezierCurve::getPosition(double value, double h) {
    return pos1 * pow(1-value, 3) + pos2 * (3*pow(1-value, 2)*value) + pos3 * (3*(1-value)*pow(value, 2)) + pos4 * pow(value, 3);
}

Position BezierCurve::getDerivative(double value) {
    return pos1 *(-3)* pow(1-value, 2) + pos2 * (-6*(1-value)*value + 3 *pow(1-value, 2)) + pos3 * (3*(-pow(value, 2) + 2*value*(1-value))) + pos4 * 3 * pow(value,2);
}