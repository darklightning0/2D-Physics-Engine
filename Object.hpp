
#ifndef OBJECT_HPP
#define OBJECT_HPP

#include "Vector.hpp"

class Object{

public:

    float radius;
    float width;
    float height;

    Vector position;
    Vector linearVelocity;
    float rotation;
    float rotationalVelocity;

    Vector force;

    float mass;
    float restitution;
    sf::Shape* shape;
    sf::Color color;
    bool isStatic;

    float density;
    float area;

    Object(float radius, float width, float height, Vector position, Vector linearVelocity, float rotation, float rotationalVelocity, Vector force, float mass, float restitution, sf::Shape* shape, sf::Color color, bool isStatic) 
        : radius(radius), width(width), height(height), position(position), linearVelocity(linearVelocity), rotation(rotation), rotationalVelocity(rotationalVelocity), force(force), mass(mass), restitution(restitution), shape(shape), color(color), isStatic(isStatic) 
    {
        
        shape->setPosition(sf::Vector2f(position.x, position.y));
        shape->setFillColor(color);

    }

    void Move(const Vector& v) const{

        this->shape->move(sf::Vector2f(v.x, v.y));

    }

};

#endif 