
#ifndef POSITION_H
#define POSITION_H
#include <optional>

#include "Angle.h"
#ifdef ARDUINO
#include "Arduino.h"
#endif
class Position : public Printable {

private:
    double x;
    double y;
    Angle a;

    public:
    virtual ~Position() = default;

    Position(double x=0.0f, double y=0.0f, Angle a=AngleConstants::ZERO);
    
    double getX() const;
    
    double getY() const;

    Angle getAngle() const;

    double getDistance() const;

    Angle getVectorAngle() const;

    void add(double x, double y, Angle a=AngleConstants::ZERO);

    Position operator+(const Position& rhs) const;

    Position operator-(const Position& rhs) const;

    Position operator+=(const Position& rhs);

    Position operator*(double rhs) const;

    Position operator/(double rhs) const;
#ifdef ARDUINO
    size_t printTo(Print& p) const override;
#endif
    Position getSinCosAngle() const;

    Position getNormalVector() const;


};
std::optional<Position> intersectPerpendicularLine(const Position& p1, const Position& p2);


#endif
