//
// Created by fogoz on 10/05/2025.
//

#include "curves/ArcCurve.h"


ArcCurve::ArcCurve(Position center, double radius, Angle angleStart, Angle angleEnd): BaseCurve(0, 1), center(center), radius(radius), angleStart(angleStart), angleEnd(angleEnd) {
}

Position ArcCurve::getPosition(double value, double h) {
    double angle = angleStart.toRadians() + value * (angleEnd.toRadians() - angleStart.toRadians());
    double x = center.getX() + radius * cos(angle);
    double y = center.getY() + radius * sin(angle);
    return Position(x, y);//Angle not used
}

Position ArcCurve::getDerivative(double value) {
    double angle = angleStart.toRadians() + value * (angleEnd.toRadians() - angleStart.toRadians());
    double dAngle = angleEnd.toRadians() - angleStart.toRadians();

    double dx = -radius * sin(angle) * dAngle;
    double dy =  radius * cos(angle) * dAngle;

    return Position(dx, dy, AngleConstants::ZERO);
}

std::optional<ArcCurve> getArcCurve(Position begin, Position end, bool backward) {
    auto o_center = intersectPerpendicularLine(begin, end);
    if(!o_center.has_value()){
        return std::nullopt;
    }
    auto center = o_center.value();
    auto d0 = begin - center;
    auto d1 = end - center;

    auto angleStart = d0.getVectorAngle() ;
    auto angleEnd = d1.getVectorAngle() ;

    auto delta = angleEnd - angleStart;
    delta.warpAngle();

    if (backward) {
        if (delta.toDegrees() < 0) {
            delta += AngleConstants::FULL_TURN;
        }
    }else {
        if (delta.toDegrees() > 0) {
            delta -= AngleConstants::FULL_TURN;
        }
    }
    return ArcCurve(center, d0.getDistance(), angleStart, angleStart+delta);
}
