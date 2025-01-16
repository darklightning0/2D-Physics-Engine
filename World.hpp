#ifndef WORLD_HPP
#define WORLD_HPP

#include "Object.hpp"

class World{

public:

    World(Vector gravity) : gravity(gravity) {}

    inline void addObject(Object* object);
    inline Object createRectangle(World& world, float width = 40.f, float height = 60.f, Vector position = Vector(100.f, 100.f), Vector linearVelocity = Vector(0, 0), float rotation = 0, float rotationalVelocity = 0, Vector force = Vector(0.f, 0.f), float mass = 10.f, float restitution = 0.5f, sf::Color color = sf::Color::Red, bool isStatic = false);
    inline Object createCircle(World& world, float radius = 20.f, Vector position = Vector(100.f, 100.f), Vector linearVelocity = Vector(0, 0), float rotation = 0, float rotationalVelocity = 0, Vector force = Vector(0.f, 0.f), float mass = 10.f, float restitution = 0.5f, sf::Color color = sf::Color::Red, bool isStatic = false);   
    inline void update(float deltaTime);

    inline float getGravity() const;
    inline const std::vector<Object*>& getObjects() const;

private:

    Vector gravity;
    std::vector<Object*> objects;

};

#endif
