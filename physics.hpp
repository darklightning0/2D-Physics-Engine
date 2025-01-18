
#ifndef PHYSICS_HPP
#define PHYSICS_HPP

#include "Object.hpp"

inline void handleTransformtions(float deltaTime, std::vector<std::shared_ptr<Object>>& objects);

inline void handleGravity(float deltaTime, const std::shared_ptr<Object>& obj, Vector gravity);

inline void handleCollisions(const std::vector<std::shared_ptr<Object>>& objects, std::shared_ptr<Object> obj1);

inline void cleaner(std::vector<std::shared_ptr<Object>>& objects, const std::shared_ptr<Object>& obj);

#endif

