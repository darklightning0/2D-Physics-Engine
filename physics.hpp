
#ifndef PHYSICS_HPP
#define PHYSICS_HPP

#include "Object.hpp"

inline void handleTransformtions(float deltaTime, std::vector<Object*> objects);

inline void handleGravity(float deltaTime, Object* obj, Vector gravity);

inline void handleCollisions(std::vector<Object*> objects, Object* obj1);

#endif

