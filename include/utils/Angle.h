//
// Created by fogoz on 08/05/2025.
//

#ifndef ANGLE_H
#define ANGLE_H
#include "Arduino.h"
#define WARP_ANGLE_DEG(angle) fmod(fmod(angle + 180, 360) - 360, 360) + 180
#define WARP_ANGLE(angle) fmod(fmod(angle + M_PI, 2*M_PI) - 2*M_PI, 2*M_PI) + M_PI

class Angle {
private:
    double rad;  // internal representation

public:
    constexpr Angle(double rad) : rad(rad) {

    }

    static constexpr Angle fromDegrees(double deg) {
        return {deg * M_PI / 180.0};
    }

    static constexpr Angle fromRadians(double rad) {
        return {rad};
    }

    double toDegrees() const {
        return rad * 180.0 / M_PI;
    }

    double toRadians() const {
        return rad;
    }

    Angle& warpAngle() {
        rad = WARP_ANGLE(rad);
        return *this;
    }

    // Comparison operators generated with the help of ia (chat.deepseek.com)
    bool operator==(const Angle& other) const {
        return std::abs(rad - other.rad) < 1e-10;  // small epsilon for floating point comparison
    }

    bool operator!=(const Angle& other) const {
        return !(*this == other);
    }

    bool operator<(const Angle& other) const {
        return rad < other.rad;
    }

    bool operator<=(const Angle& other) const {
        return rad <= other.rad;
    }

    bool operator>(const Angle& other) const {
        return rad > other.rad;
    }

    bool operator>=(const Angle& other) const {
        return rad >= other.rad;
    }

    // Arithmetic operators
    Angle operator+(const Angle& other) const {
        return {rad + other.rad};
    }

    Angle operator-(const Angle& other) const {
        return {rad - other.rad};
    }

    Angle operator-() const {
        return {-rad};
    }

    Angle& operator+=(const Angle& other) {
        rad += other.rad;
        return *this;
    }

    Angle& operator-=(const Angle& other) {
        rad -= other.rad;
        return *this;
    }

    // Scalar multiplication/division
    Angle operator*(double scalar) const {
        return {rad * scalar};
    }

    Angle operator/(double scalar) const {
        return {rad / scalar};
    }

    Angle& operator*=(double scalar) {
        rad *= scalar;
        return *this;
    }

    Angle& operator/=(double scalar) {
        rad /= scalar;
        return *this;
    }
};

// User-defined literals
inline Angle operator"" _deg(long double deg) {
    return Angle::fromDegrees(static_cast<double>(deg));
}

inline Angle operator"" _rad(long double rad) {
    return Angle::fromRadians(static_cast<double>(rad));
}

inline Angle operator"" _deg(unsigned long long deg) {  // for integer degrees
    return Angle::fromDegrees(static_cast<double>(deg));
}

inline Angle operator"" _rad(unsigned long long rad) {  // for integer radians
    return Angle::fromRadians(static_cast<double>(rad));
}


// Scalar multiplication (commutative)
inline Angle operator*(double scalar, const Angle& angle) {
    return angle * scalar;
}
#undef PI
namespace AngleConstants {
    inline constexpr Angle ZERO = Angle::fromDegrees(0);
    inline constexpr Angle FRONT = Angle::fromDegrees(0);
    inline constexpr Angle LEFT = Angle::fromDegrees(90);
    inline constexpr Angle RIGHT = Angle::fromDegrees(-90);
    inline constexpr Angle BACK = Angle::fromDegrees(180);
}


#endif //ANGLE_H
