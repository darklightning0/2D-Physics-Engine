#ifndef WORLD_HPP
#define WORLD_HPP

#include "physics.hpp"

class World{

public:

    World(Vector gravity) : gravity(gravity) {}

    inline void addObject(Object* object);
    inline Object& createRectangle(World& world, float width = 40.f, float height = 60.f, Vector position = Vector(100.f, 100.f), float rotation = 0, float density = 1.f, float mass = 1.f, float restitution = 0.5f, sf::Color color = sf::Color::Red, bool isStatic = false);
    inline Object& createCircle(World& world, float radius = 20.f, Vector position = Vector(100.f, 100.f), float rotation = 0, float density = 1.f, float mass = 1.f, float restitution = 0.5f, sf::Color color = sf::Color::Red, bool isStatic = false);   
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
