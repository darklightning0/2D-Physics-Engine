#ifndef WORLD_HPP
#define WORLD_HPP

#include "physics.hpp"
#include "Material.hpp"
#include "Joint.hpp"
#include "BroadPhase.hpp"
#include "CollisionCache.hpp"
#include <unordered_map>
#include <string>

class World{

public:

    World(Vector gravity) : gravity(gravity) {}

    inline void addObject(Object* object);
    inline Object& createRectangle(World& world, float width, float height, Vector position, Vector linearVelocity, float angle, float angularVelocity, float mass, float restitution = -1.f, sf::Color color = sf::Color::White, bool isStatic = false, Material material = Material::Default);
    inline Object& createCircle(World& world, float radius, Vector position, Vector linearVelocity, float angle, float angularVelocity, float mass, float restitution = -1.f, sf::Color color = sf::Color::White, bool isStatic = false, Material material = Material::Default);   
    inline Object& createPolygon(World& world, const std::vector<Vector>& localVertices, Vector position, Vector linearVelocity, float angle, float angularVelocity, float mass, float restitution = -1.f, sf::Color color = sf::Color::White, bool isStatic = false, Material material = Material::Default);
    inline DistanceJoint& createDistanceJoint(Object& objA, Object& objB, const Vector& localAnchorA, const Vector& localAnchorB, float restLength, float frequencyHz = 2.f, float dampingRatio = 0.7f);
    inline SpringJoint& createSpringJoint(Object& objA, Object& objB, const Vector& localAnchorA, const Vector& localAnchorB, float restLength, float stiffness, float damping);
    inline void update(float deltaTime, int iterations);
    inline void removeObject(Object* object);
    inline void clear();
    bool saveToFile(const std::string& path) const;
    bool loadFromFile(const std::string& path);

    inline float getGravity() const;
    inline const std::vector<Object*>& getObjects() const;
    inline const std::vector<Manifold*>& getContacts() const;
    inline const std::vector<Manifold*>& getIndicators() const;
    inline const std::vector<DistanceJoint*>& getDistanceJoints() const;
    inline const std::vector<SpringJoint*>& getSpringJoints() const;

private:

    Vector gravity;
    std::vector<Object*> objects;
    std::vector<Manifold*> contactList;
    std::vector<Manifold*> indicatorList;
    std::vector<std::pair<Object*, Object*>> potentialPairs;
    std::unordered_map<PairKey, CachedManifold, PairKeyHash> manifoldCache;
    std::vector<DistanceJoint*> distanceJoints;
    std::vector<SpringJoint*> springJoints;

    inline void removeJointsForObject(Object* object);

};

#endif
