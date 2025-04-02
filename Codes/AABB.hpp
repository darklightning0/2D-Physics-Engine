
#ifndef AABB_HPP
#define AABB_HPP

#include "Transform.hpp"

class AABB{

public:

    Vector min;
    Vector max;

    AABB() : min(Vector(0, 0)), max(Vector(0, 0)){}

    AABB(Vector min, Vector max) : min(min), max(max){}

    AABB(float minX, float minY, float maxX, float maxY){

        this->min = Vector(minX, minY);
        this->max = Vector(maxX, maxY);

    }

};

#endif 