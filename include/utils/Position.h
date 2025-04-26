
#ifndef POSITION_H
#define POSITION_H
#include "Arduino.h"

#define WARP_ANGLE_DEG(angle) fmod(fmod(angle + 180, 360) - 360, 360) + 180
#define WARP_ANGLE(angle) fmod(fmod(angle + M_PI, 2*M_PI) - 2*M_PI, 2*M_PI) + M_PI
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
