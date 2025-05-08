#include "utils/Position.h"

Position::Position(double x, double y, Angle a) : x(x), y(y), a(a){
    a.warpAngle();
}

double Position::getX() const{
    return x;
}

double Position::getY() const{
    return y;
}

Angle Position::getAngle() const{
    return a;
}


void Position::add(double x, double y, Angle a){
    this->x += x;
    this->y += y;
    this->a += a;
    this->a.warpAngle();
}


double Position::getDistance() const {
    return sqrt(pow(this->x, 2) + pow(this->y, 2));
}
size_t Position::printTo(Print &p) const {
    size_t length = 0;
    length += p.print("x: ");
    length += p.print(x);
    length += p.print(", y: ");
    length += p.print(y);
    length += p.print(", angle: ");
    length += p.print(a.toDegrees());
    return length;
}

Position Position::operator+(const Position& pos) const{
  return {this->x + pos.x, this->y + pos.y, this->a+ pos.a};
}

Position Position::operator-(const Position& pos) const{
  return {this->x - pos.x, this->y - pos.y, this->a - pos.a};
}

Position Position::operator+=(const Position &rhs) {
    this->x += rhs.x;
    this->y += rhs.y;
    this->a += rhs.a;
    this->a.warpAngle();
    return *this;
}

Position Position::operator*(const double rhs) const {
    return {this->x * rhs, this->y * rhs, this->a * rhs};
}

Position Position::operator/(const double rhs) const{
    return {this->x / rhs, this->y / rhs, this->a / rhs};
}

Angle Position::getVectorAngle() const {
    return Angle::fromRadians(atan2(this->y, this->x));
}

Position Position::getSinCosAngle() const {
    return {cos(this->a.toRadians()), sin(this->a.toRadians())};
}