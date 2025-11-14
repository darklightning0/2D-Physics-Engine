#ifndef WORLD_HPP
#define WORLD_HPP

#include "physics.hpp"
#include "Material.hpp"
#include "Joint.hpp"
#include "BroadPhase.hpp"
#include "CollisionCache.hpp"
#include <unordered_map>

class World{

public:

    World(Vector gravity) : gravity(gravity) {}

    inline void addObject(Object* object);
    inline Object& createRectangle(World& world, float width, float height, Vector position, Vector linearVelocity, float angle, float angularVelocity, float mass, float restitution = -1.f, sf::Color color = sf::Color::White, bool isStatic = false, Material material = Material::Default);
    inline Object& createCircle(World& world, float radius, Vector position, Vector linearVelocity, float angle, float angularVelocity, float mass, float restitution = -1.f, sf::Color color = sf::Color::White, bool isStatic = false, Material material = Material::Default);   
    inline DistanceJoint& createDistanceJoint(Object& objA, Object& objB, const Vector& localAnchorA, const Vector& localAnchorB, float restLength, float frequencyHz = 2.f, float dampingRatio = 0.7f);
    inline void update(float deltaTime, int iterations);

    inline float getGravity() const;
    inline const std::vector<Object*>& getObjects() const;
    inline const std::vector<Manifold*>& getContacts() const;
    inline const std::vector<Manifold*>& getIndicators() const;
    inline const std::vector<DistanceJoint*>& getDistanceJoints() const;

private:

    Vector gravity;
    std::vector<Object*> objects;
    std::vector<Manifold*> contactList;
    std::vector<Manifold*> indicatorList;
    std::vector<std::pair<Object*, Object*>> potentialPairs;
    std::unordered_map<PairKey, CachedManifold, PairKeyHash> manifoldCache;
    std::vector<DistanceJoint*> distanceJoints;

    inline void removeJointsForObject(Object* object);

};

#endif
