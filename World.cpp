
#include "World.hpp"
#include "physics.cpp"

void World::addObject(std::shared_ptr<Object> object){

    this->objects.push_back(object);

}

std::shared_ptr<Object> World::createRectangle(World& world, float width, float height, Vector position, Vector linearVelocity, float rotation, float rotationalVelocity, Vector force, float mass, float restitution, sf::Color color, bool isStatic){

    auto RectShape = std::make_unique<sf::RectangleShape>(sf::Vector2f(width, height));
    RectShape->setOrigin(width / 2.0f, height / 2.0f);
    RectShape->setOutlineThickness(2.f);     
    RectShape->setOutlineColor(sf::Color::White);
    auto RectObject = std::make_shared<Object>(0, width, height, position, linearVelocity, rotation, rotationalVelocity, force, mass, restitution, std::move(RectShape), color, isStatic, 1);
    world.addObject(RectObject);

    return RectObject;

}

std::shared_ptr<Object> World::createCircle(World& world, float radius, Vector position, Vector linearVelocity, float rotation, float rotationalVelocity, Vector force, float mass, float restitution, sf::Color color, bool isStatic){

    auto CircShape =  std::make_unique<sf::CircleShape>(radius);
    CircShape->setOrigin(radius, radius);
    CircShape->setOutlineThickness(2.f);  
    CircShape->setOutlineColor(sf::Color::White);
    auto CircObject = std::make_shared<Object>(radius, 0, 0, position, linearVelocity, rotation, rotationalVelocity, force, mass, restitution, std::move(CircShape), color, isStatic, 0);
    world.addObject(CircObject);

    return CircObject;

}

void World::update(float deltaTime){

    for(auto& obj : objects){

        handleGravity(deltaTime, obj, gravity);
        handleCollisions(objects, obj);
        cleaner(objects, obj);

    }

    handleTransformtions(deltaTime, objects);
    
}

float World::getGravity() const{

    return gravity.y;

}

const std::vector<std::shared_ptr<Object>>& World::getObjects() const{

    return objects;
    
}
