
#ifndef POSITION_H
#define POSITION_H
#include "Arduino.h"

class Position : public Printable {

private:
    double x;
    double y;
    double a;

    public:
    Position(double x=0.0f, double y=0.0f, double a=0.0f);
    
    double getX() const;
    
    double getY() const;

    double getAngle() const;

    double getDistance() const;

    double getVectorAngle() const;

    void add(double x, double y, double a=0.0f);

    Position operator+(const Position& rhs) const;

    Position operator-(const Position& rhs) const;

    Position operator+=(const Position& rhs);

    Position operator*(double rhs) const;

    Position operator/(double rhs) const;

    size_t printTo(Print& p) const override;

    double getAngleRad() const;

    double getVectorAngleRad() const;

    Position getSinCosAngle() const;


};



#endif
