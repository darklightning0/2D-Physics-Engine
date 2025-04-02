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

    Object(float radius, float width, float height, Vector position, float density, float mass, float restitution,
        sf::Shape* shape, sf::Color color, bool isStatic, int type)
        : radius(radius), width(width), height(height), position(position), linearVelocity(Vector::Zero()),
          rotation(0.f), rotationalVelocity(0.f), force(Vector::Zero()), mass(1.f), restitution(0.5),
          density(1.f), isStatic(isStatic), shape(shape), color(color), type(type),
          transformUpdateRequired(true), aabbUpdateRequired(true){

        this->MoI = this->getMomentOfInertia();

        if(this->isStatic){

            this->invMass = 0;
            this->invMoI = 0;

        }else{

            this->invMass = 1.f / this->mass;
            this->invMoI = 1.f / this->invMoI;

        }

        if (type == 1){ // Box

            vertices = createBoxVertices(width, height);
            triangles = createBoxTriangles();
            transformedVertices.resize(vertices.size());

        }else { // Circle or other

            vertices.clear();
            triangles.clear();
            transformedVertices.clear();
        }

        if(shape){
            
            shape->setPosition(sf::Vector2f(position.x, position.y));
            shape->setFillColor(color);
        }

    }

    float getMomentOfInertia(){

        if(this->type == 0){

            return 0.5 * this->mass * this->radius;

        }else{

            return 1/12 * this->mass * (this->height * this->height + this->width * this->width);

        }

    }

    void Move(const Vector& v) {
        position = position + v;
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

    std::vector<int> createBoxTriangles() const {
        return {0, 1, 2, 0, 2, 3};
    }

    std::vector<Vector> getTransformedVertices() {
        if (transformUpdateRequired) {
            Modify modify(position, rotation);

            for (size_t i = 0; i < vertices.size(); ++i) {
                transformedVertices[i] = ApplyTransform(vertices[i], modify);
            }

        }

        transformUpdateRequired = false;
        return transformedVertices;

    }

    AABB getAABB() {
        if (aabbUpdateRequired) {
            float minX = std::numeric_limits<float>::max();
            float minY = std::numeric_limits<float>::max();
            float maxX = std::numeric_limits<float>::lowest();
            float maxY = std::numeric_limits<float>::lowest();

            if (type == 1) { // Box
                auto transformed = getTransformedVertices();

                for (const auto& v : transformed) {
                    minX = std::min(minX, v.x);
                    maxX = std::max(maxX, v.x);
                    minY = std::min(minY, v.y);
                    maxY = std::max(maxY, v.y);
                }
            } else if (type == 0) { // Circle
                minX = position.x - radius;
                minY = position.y - radius;
                maxX = position.x + radius;
                maxY = position.y + radius;
            } else {
                throw std::runtime_error("Unknown shape type.");
            }

            aabb = AABB(minX, minY, maxX, maxY);
            aabbUpdateRequired = false;
        }

        return aabb;
    }
};

#endif
