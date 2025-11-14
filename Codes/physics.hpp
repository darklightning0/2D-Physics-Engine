
#ifndef PHYSICS_HPP
#define PHYSICS_HPP

#include "utility.hpp"
#include "CollisionCache.hpp"
#include <unordered_map>

inline void handleTransformations(float deltaTime, std::vector<Object*> objects);

inline void handleGravity(float deltaTime, Object* obj, Vector gravity);

inline void handleCollisions(const std::vector<std::pair<Object*, Object*>>& pairs, std::vector<Manifold*>& contactList, std::unordered_map<PairKey, CachedManifold, PairKeyHash>& manifoldCache);

inline void cleaner(std::vector<Object*>& objects, Object& obj);

#endif
