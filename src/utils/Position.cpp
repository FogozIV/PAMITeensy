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
#ifdef ARDUINO
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
#endif

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

Position Position::getNormalVector() const {
    return {-sin(this->a.toRadians()), cos(this->a.toRadians())};
}

std::optional<Position> intersectLines(const Position&p1, const Position&n1, const Position&p2, const Position&n2) {
    double det = -n1.getX()*n2.getY() + n1.getY()*n2.getX();
    if (fabs(det) < 1e-6) {
        return std::nullopt;
    }
    double dx = p1.getX() - p2.getX();
    double dy = p1.getY() - p2.getY();

    double s = (dx * -n2.getY() + dy * n2.getX()) / det;
    Position center = {p1.getX() + s * n1.getX(), p1.getY() + s * n1.getY()};
    return center;
}

std::optional<Position> intersectPerpendicularLine(const Position &p1, const Position &p2) {
    Position n1 = p1.getNormalVector();
    Position n2 = p2.getNormalVector();
    return intersectLines(p1, n1, p2, n2);
}
