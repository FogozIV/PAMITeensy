#include "utils/Position.h"

Position::Position(double x, double y, double a) : x(x), y(y), a(a){
    while(this->a > M_PI)
        this->a -= 2*M_PI;
    while(this->a < -M_PI)
        this->a += 2*M_PI;
}

double Position::getX() const{
    return x;
}

double Position::getY() const{
    return y;
}

double Position::getAngle() const{
    return a*180/M_PI;
}

double Position::getAngleRad() const {
    return a;
}

void Position::add(double x, double y, double a){
  this->x += x;
  this->y += y;
  this->a += a;
  while(this->a > M_PI)
    this->a -= 2*M_PI;
  while(this->a < -M_PI)
    this->a += 2*M_PI;
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
    length += p.print(a*180/M_PI);
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
    while(this->a > M_PI)
        this->a -= 2*M_PI;
    while(this->a < -M_PI)
        this->a += 2*M_PI;
    return *this;
}

Position Position::operator*(const double rhs) const {
    return {this->x * rhs, this->y * rhs, this->a * rhs};
}

Position Position::operator/(const double rhs) const{
    return {this->x / rhs, this->y / rhs, this->a / rhs};
}

double Position::getVectorAngle() const {
    return atan2(this->y, this->x) / M_PI * 180;
}

double Position::getVectorAngleRad() const {
    return atan2(this->y, this->x) ;
}

Position Position::getSinCosAngle() const {
    return {cos(this->a), sin(this->a)};
}