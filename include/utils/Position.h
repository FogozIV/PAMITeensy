
#ifndef POSITION_H
#define POSITION_H
#include "Angle.h"
#include "Arduino.h"

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

    size_t printTo(Print& p) const override;

    Position getSinCosAngle() const;


};



#endif
