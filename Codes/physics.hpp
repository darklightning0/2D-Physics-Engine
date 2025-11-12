
#ifndef PHYSICS_HPP
#define PHYSICS_HPP

#include "utility.hpp"

inline void handleTransformations(float deltaTime, std::vector<Object*> objects);

inline void handleGravity(float deltaTime, Object* obj, Vector gravity);

inline void handleCollisions(std::vector<Object*>& objects, Object* obj1, std::vector<Manifold*>& contactList, std::vector<Manifold*>& indicatorList);

inline void cleaner(std::vector<Object*>& objects, Object& obj);

#endif

