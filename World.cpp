
#include "World.hpp"
#include "physics.cpp"

void World::addObject(Object* object){

    objects.push_back(object);

}

void World::createRectangle(World& world, float width, float height, Vector position, Vector linearVelocity, float rotation, float rotationalVelocity, Vector force, float mass, float restitution, sf::Color color, bool isStatic){

    RectangleShape* RectShape = new RectangleShape(Vector2f(width, height)); 
    Object* RectObject = new Object(0, width, height, position, linearVelocity, rotation, rotationalVelocity, force, mass, restitution, RectShape, color, isStatic);
    world.addObject(RectObject);

}

void World::createCircle(World& world, float radius, Vector position, Vector linearVelocity, float rotation, float rotationalVelocity, Vector force, float mass, float restitution, sf::Color color, bool isStatic){

    CircleShape* CircShape = new CircleShape(20.f);
    Object* CircObject = new Object(radius, 0, 0, position, linearVelocity, rotation, rotationalVelocity, force, mass, restitution, CircShape, color, isStatic);
    world.addObject(CircObject);

}

void World::update(float deltaTime){

    for(Object*& obj : objects){

        handleGravity(deltaTime, obj, gravity);
        handleCollisions(objects, obj);

    }

    handleTransformtions(deltaTime, objects);
    
}

float World::getGravity() const{

    return gravity.y;

}

const std::vector<Object*>& World::getObjects() const{

    return objects;

}
