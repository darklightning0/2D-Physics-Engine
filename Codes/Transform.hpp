#ifndef TRANSFORM_HPP
#define TRANSFORM_HPP

#include "Vector.hpp"
#include <cmath>

class Modify {
public:
    float positionX;
    float positionY;
    float sinAngle;
    float cosAngle;

    static const Modify Zero;

    Modify(const Vector& position, float angle)
        : positionX(position.x),
          positionY(position.y),
          sinAngle(std::sin(-angle)),
          cosAngle(std::cos(-angle)) {}

    Modify(float x, float y, float angle)
        : positionX(x),
          positionY(y),
          sinAngle(std::sin(angle)),
          cosAngle(std::cos(angle)) {}
};

inline const Modify Modify::Zero = Modify(0.f, 0.f, 0.f);

namespace TransformUtils {
    inline Vector ApplyTransform(const Vector& v, const Modify& modify) {
        return Vector(
            (modify.cosAngle * v.x - modify.sinAngle * v.y) + modify.positionX,
            (modify.sinAngle * v.x + modify.cosAngle * v.y) + modify.positionY
        );
    }
}

#endif
