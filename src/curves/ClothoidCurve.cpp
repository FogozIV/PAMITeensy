//
// Created by fogoz on 10/05/2025.
//

#include "curves/ClothoidCurve.h"

ClothoidCurve::ClothoidCurve(Position start, double startCurvature, double a, double length) : BaseCurve(0, length), start(start), startCurvature(startCurvature), a(a), length(length) {
}

Position ClothoidCurve::getPosition(double value, double h) {
    if (std::get<0>(previousPos) == value) {
        return std::get<1>(previousPos);
    }
    std::vector<double> y = {0.0,0.0, start.getAngle().toRadians()};

    runge_kutta_4(0.0, y, value, h, [this](double t, std::vector<double> &y, std::vector<double> &dydt) {
        this->clothoidODE(t, y, dydt);
    });

    double startAngle = start.getAngle().toRadians();
    double cosAngle = cos(startAngle);
    double sinAngle = sin(startAngle);

    double rotatedX = y[0] * cosAngle - y[1] * sinAngle;
    double rotatedY = y[0] * sinAngle + y[1] * cosAngle;
    auto pos = Position(start.getX() + rotatedX, start.getY() + rotatedY, Angle::fromRadians(y[2]));
    previousPos = std::make_tuple(value, pos);
    return pos;
}

void ClothoidCurve::clothoidODE(double t, std::vector<double> &y, std::vector<double> &dydt) const{
    double current_curvature = startCurvature + a * t;

    dydt[0] = cos(y[2]);           // dx/dt
    dydt[1] = sin(y[2]);           // dy/dt
    dydt[2] = current_curvature;   // dtheta/dt
}

Position ClothoidCurve::getDerivative(double value)  {
    if (std::get<0>(previousDerivative) == value) {
        return std::get<1>(previousDerivative);
    }

    std::vector<double> y = {
        0.0,  // x
        0.0,  // y
        start.getAngle().toRadians()  // θ
    };
    std::vector<double> dydt(3);
    clothoidODE(value, y, dydt);
    auto pos = Position(dydt[0], dydt[1], Angle::fromRadians(dydt[2]));
    previousDerivative = std::make_tuple(value, pos);
    return pos;
}

double ClothoidCurve::getLength(double ti, double t_end, double h) {
    return t_end - ti;
}

double ClothoidCurve::getValueForLength(double ti, double length, double h) {
    return min(ti + length, length);
}
