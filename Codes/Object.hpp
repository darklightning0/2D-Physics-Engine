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
    float rotation;
    float rotationalVelocity;

    Vector force;

    float restitution;
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
    std::vector<int> triangles;
    std::vector<Vector> transformedVertices;
    AABB aabb;
    bool transformUpdateRequired;
    bool aabbUpdateRequired;

    Object(float radius, float width, float height, Vector position, float rotation, float density, float mass, float restitution,
        sf::Shape* shape, sf::Color color, bool isStatic, int type)
        : radius(radius), width(width), height(height), position(position), linearVelocity(Vector::Zero()),
          rotation(0.f), rotationalVelocity(0.f), force(Vector::Zero()), mass(1.f), restitution(0.5),
          density(1.f), isStatic(isStatic), shape(shape), color(color), type(type),
          transformUpdateRequired(true), aabbUpdateRequired(true){

        MoI = getMomentOfInertia();

        if(isStatic){

            invMass = 0;
            this->invMoI = 0;

        }else{

            invMass = 1.f / mass;
            invMoI = 1.f / MoI;

        }

        if(type == 1){ 

            vertices = createBoxVertices(width, height);
            triangles = createBoxTriangles();
            transformedVertices.resize(vertices.size());

        }else{

            vertices.clear();
            triangles.clear();
            transformedVertices.clear();
        }

        Rotate(rotation);

        if(shape){
            
            shape->setPosition(sf::Vector2f(position.x, position.y));
            shape->setFillColor(color);

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
        transformUpdateRequired = true;
        aabbUpdateRequired = true;

    }

    void Rotate(float amount){

        rotation += amount;
        shape->rotate(-amount);
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

    std::vector<int> createBoxTriangles() const{

        return {0, 1, 2, 0, 2, 3};
    
    }

    std::vector<Vector> getTransformedVertices(){

        if(transformUpdateRequired){

            Modify modify(position, rotation);

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
