#ifndef WORLD_HPP
#define WORLD_HPP

#include "physics.hpp"

class World{

public:

    World(Vector gravity) : gravity(gravity) {}

    inline void addObject(Object* object);
    inline Object& createRectangle(World& world, float width, float height, Vector position, Vector linearVelocity, float angle, float angularVelocity, float mass, float restitution, sf::Color color, bool isStatic, float staticFriction = 0.5f, float dynamicFriction = 0.3f);
    inline Object& createCircle(World& world, float radius, Vector position, Vector linearVelocity, float angle, float angularVelocity, float mass, float restitution, sf::Color color, bool isStatic, float staticFriction = 0.5f, float dynamicFriction = 0.3f);   
    inline void update(float deltaTime, int iterations);

    inline float getGravity() const;
    inline const std::vector<Object*>& getObjects() const;
    inline const std::vector<Manifold*>& getContacts() const;
    inline const std::vector<Manifold*>& getIndicators() const;

private:

    Vector gravity;
    std::vector<Object*> objects;
    std::vector<Manifold*> contactList;
    std::vector<Manifold*> indicatorList;

};

#endif
