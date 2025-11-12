#ifndef OBJECT_HPP
#define OBJECT_HPP

#include "AABB.hpp"
#include "Vector.hpp"
#include "Transform.hpp"
#include <vector>
#include <limits>
#include <algorithm>
#include <stdexcept>

class Object {
public:

    float radius;
    float width;
    float height;

    Vector position;
    Vector linearVelocity;
    float angle;
    float angularVelocity;

    Vector force;

    float restitution;
    float staticFriction;
    float dynamicFriction;
    sf::Shape* shape;
    sf::Color color;
    bool isStatic;
    int type;

    float mass;
    float invMass;
    float MoI;
    float invMoI;

    float density;

    std::vector<Vector> vertices;
    std::vector<Vector> transformedVertices;
    AABB aabb;
    bool transformUpdateRequired;
    bool aabbUpdateRequired;

    Object(float radius, float width, float height, Vector position, Vector linearVelocity, float angle, float angularVelocity,
        float mass, float restitution, sf::Shape* shape, sf::Color color, bool isStatic, int type,
        float staticFriction = 0.5f, float dynamicFriction = 0.3f)
        : radius(radius), width(width), height(height), position(position), linearVelocity(linearVelocity),
          angle(angle), angularVelocity(angularVelocity), force(Vector::Zero()), mass(mass), restitution(restitution),
          staticFriction(staticFriction), dynamicFriction(dynamicFriction), shape(shape), color(color), isStatic(isStatic), type(type),
          transformUpdateRequired(true), aabbUpdateRequired(true){

        MoI = getMomentOfInertia();

        if(isStatic){

            invMass = 0;
            invMoI = 0;

        }else{

            invMass = 1.f / mass;
            invMoI = 1.f / MoI;

        }

        if(type == 1){ 

            vertices = createBoxVertices(width, height);
            transformedVertices.resize(vertices.size());
            density = mass / (width * height);

        }else{

            vertices.clear();
            transformedVertices.clear();

            density = mass / (M_PI * radius * radius);

        }

        if(shape){
            
            shape->setPosition(sf::Vector2f(position.x, position.y));
            shape->setFillColor(color);
            shape->setRotation(-Vector::radToAngle(angle));

        }

    }

    Vector getPosition(){

        return Vector(shape->getPosition().x, shape->getPosition().y);

    }

    float getMomentOfInertia(){

        if(type == 0){

            return 0.5 * mass * radius * radius;

        }else{

            return (1.f/12.f) * mass * (height * height + width * width);

        }

    }

    void Move(const Vector& v){

        position = position + v;

        if(shape){

            shape->setPosition(position.x, position.y);

        }

        transformUpdateRequired = true;
        aabbUpdateRequired = true;

    }

    void MoveTo(const Vector& v){

        position = v;

        if(shape){

            shape->setPosition(position.x, position.y);

        }

        transformUpdateRequired = true;
        aabbUpdateRequired = true;

    }

    void Rotate(float amount){

        angle += amount;

        if(shape){

            shape->rotate(-Vector::radToAngle(amount));

        }

        transformUpdateRequired = true;
        aabbUpdateRequired = true;

    }

    std::vector<Vector> createBoxVertices(float width, float height) const {
        float left = -width / 2.f;
        float right = left + width;
        float bottom = -height / 2.f;
        float top = bottom + height;

        return {
            Vector(left, top),
            Vector(right, top),
            Vector(right, bottom),
            Vector(left, bottom)
        };
    }

    std::vector<Vector> getTransformedVertices(){

        if(transformUpdateRequired){

            Modify modify(position, angle);

            for (size_t i = 0; i < vertices.size(); ++i){

                transformedVertices[i] = TransformUtils::ApplyTransform(vertices[i], modify);

            }

            transformUpdateRequired = false; 

        }

        return transformedVertices;
    }

    AABB getAABB(){

        if (aabbUpdateRequired){

            float minX = std::numeric_limits<float>::max();
            float minY = std::numeric_limits<float>::max();
            float maxX = std::numeric_limits<float>::lowest();
            float maxY = std::numeric_limits<float>::lowest();
    
            if (type == 1){

                std::vector<Vector> vertices = getTransformedVertices();
    
                for (size_t i = 0; i < vertices.size(); i++){

                    Vector v = vertices[i];
    
                    if(v.x < minX) {minX = v.x;}
                    if(v.x > maxX) {maxX = v.x;}
                    if(v.y < minY) {minY = v.y;}
                    if(v.y > maxY) {maxY = v.y;}

                }

            }else if(this->type == 0){
                
                minX = this->position.x - this->radius;
                minY = this->position.y - this->radius;
                maxX = this->position.x + this->radius;
                maxY = this->position.y + this->radius;

            }
    
            aabb = AABB(minX, minY, maxX, maxY);
            aabbUpdateRequired = false;

        }
    
        return aabb;
    }
    

};

#endif
