
#include "World.hpp"
#include "physics.cpp"

void World::addObject(Object* object){

    objects.push_back(object);

}

Object World::createRectangle(World& world, float width, float height, Vector position, Vector linearVelocity, float rotation, float rotationalVelocity, Vector force, float mass, float restitution, sf::Color color, bool isStatic){

    RectangleShape* RectShape = new RectangleShape(Vector2f(width, height)); 
    RectShape->setOrigin(width / 2.0f, height / 2.0f);
    RectShape->setOutlineThickness(2.f);     
    RectShape->setOutlineColor(sf::Color::White);
    Object* RectObject = new Object(0, width, height, position, linearVelocity, rotation, rotationalVelocity, force, mass, restitution, RectShape, color, isStatic, 1);
    world.addObject(RectObject);

    return *RectObject;

}

Object World::createCircle(World& world, float radius, Vector position, Vector linearVelocity, float rotation, float rotationalVelocity, Vector force, float mass, float restitution, sf::Color color, bool isStatic){

    CircleShape* CircShape = new CircleShape(radius);
    CircShape->setOrigin(radius, radius);
    CircShape->setOutlineThickness(2.f);  
    CircShape->setOutlineColor(sf::Color::White);
    Object* CircObject = new Object(radius, 0, 0, position, linearVelocity, rotation, rotationalVelocity, force, mass, restitution, CircShape, color, isStatic, 0);
    world.addObject(CircObject);

    return *CircObject;

}

void World::update(float deltaTime){

    for(Object*& obj : objects){

        handleGravity(deltaTime, obj, gravity);
        handleCollisions(objects, obj);
        cleaner(objects, *obj);

    }

    handleTransformtions(deltaTime, objects);
    
}

float World::getGravity() const{

    return gravity.y;

}

const std::vector<Object*>& World::getObjects() const{

    return objects;

}
