#ifndef MANIFOLD_HPP
#define MANIFOLD_HPP

#include "Object.hpp"   

class Manifold{

public:

    Object* obj1;
    Object* obj2;
    const Vector normal;
    const float depth;
    const Vector contact1;
    const Vector contact2;
    const int contactCount;

    Manifold(Object* obj1, Object* obj2, 
             const Vector& normal, float depth, 
             const Vector& contact1, const Vector& contact2, int contactCount)
        : obj1(obj1), obj2(obj2), 
          normal(normal), depth(depth), 
          contact1(contact1), contact2(contact2), contactCount(contactCount){}

};

#endif 
