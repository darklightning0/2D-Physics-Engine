
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
    int type;

    float density;
    float area;

    Object(float radius, float width, float height, Vector position, Vector linearVelocity, float rotation, float rotationalVelocity, Vector force, float mass, float restitution, sf::Shape* shape, sf::Color color, bool isStatic, int type) 
        : radius(radius), width(width), height(height), position(position), linearVelocity(linearVelocity), rotation(rotation), rotationalVelocity(rotationalVelocity), force(force), mass(mass), restitution(restitution), shape(shape), color(color), isStatic(isStatic), type(type)
    {
        
        shape->setPosition(sf::Vector2f(position.x, position.y));
        shape->setFillColor(color);

    }

    ~Object(){

        if(shape){

            delete shape;
            shape = nullptr;

        }

    }

    float getInvMass() const{

        if(isStatic == false){

            return (1.f / mass);

        }else{

            return 0.f;

        }

    }

    void Move(const Vector& v){

        this->position = this->position + v;

    }

};

#endif 