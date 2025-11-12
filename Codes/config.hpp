#ifndef CONFIG_HPP
#define CONFIG_HPP

#include "Vector.hpp"

constexpr float PIXELS_PER_METER = 60.f;
constexpr float MIN_TIME_SCALE = 0.1f;
constexpr float MAX_TIME_SCALE = 3.f;
constexpr float TIME_SCALE_STEP = 0.1f;

inline float metersToPixels(float meters){

    return meters * PIXELS_PER_METER;

}

inline float pixelsToMeters(float pixels){

    return pixels / PIXELS_PER_METER;

}

inline Vector metersToPixels(const Vector& meters){

    return Vector(meters.x * PIXELS_PER_METER, meters.y * PIXELS_PER_METER);

}

inline Vector pixelsToMeters(const Vector& pixels){

    return Vector(pixels.x / PIXELS_PER_METER, pixels.y / PIXELS_PER_METER);

}

#endif
