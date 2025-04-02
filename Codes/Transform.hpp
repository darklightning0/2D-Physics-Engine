#ifndef TRANSFORM_HPP
#define TRANSFORM_HPP

#include "Vector.hpp"
#include <cmath>

class Modify{
public:
    const float positionX;
    const float positionY;
    const float sinAngle;
    const float cosAngle; 

    static const  Modify Zero;

    Modify(const Vector& position, float angle)
        : positionX(position.x),
          positionY(position.y),
          sinAngle(std::sin(angle)),
          cosAngle(std::cos(angle)) {}

    Modify(float x, float y, float angle)
        : positionX(x),
          positionY(y),
          sinAngle(std::sin(angle)),
          cosAngle(std::cos(angle)) {}
};

// Define the static member `Zero`
static inline const Modify Zero = Modify(0.f, 0.f, 0.f);

// Transform function to apply transformations on vectors
static Vector ApplyTransform(const Vector& v, const Modify& Modify) {
    return Vector(
        Modify.cosAngle * v.x - Modify.sinAngle * v.y + Modify.positionX,
        Modify.sinAngle * v.x + Modify.cosAngle * v.y + Modify.positionY
    );
}

#endif
