#ifndef OBJECT_HPP
#define OBJECT_HPP

#include "AABB.hpp"
#include "Vector.hpp"
#include "Transform.hpp"
#include "Material.hpp"
#include <vector>
#include <limits>
#include <algorithm>
#include <stdexcept>
#include <cmath>

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
    Material material;
    sf::Shape* shape;
    sf::Color color;
    bool isStatic;
    int type;
    bool isBox;

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
        float mass, float restitution, Material material, sf::Shape* shape, sf::Color color, bool isStatic, int type, bool boxShape = false)
        : radius(radius), width(width), height(height), position(position), linearVelocity(linearVelocity),
          angle(angle), angularVelocity(angularVelocity), force(Vector::Zero()), mass(mass), restitution(restitution),
          material(material), shape(shape), color(color), isStatic(isStatic), type(type), isBox(boxShape),
          transformUpdateRequired(true), aabbUpdateRequired(true){

        if(type == 1){

            if(isBox){
                setVertices(createBoxVertices(width, height));
            }else{
                vertices.clear();
                transformedVertices.clear();
            }

        }else{

            vertices.clear();
            transformedVertices.clear();

        }

        if(shape){
            
            shape->setPosition(sf::Vector2f(position.x, position.y));
            shape->setFillColor(color);
            shape->setRotation(-Vector::radToAngle(angle));

        }

        updateDerivedProperties();

    }

    Vector getPosition(){

        return Vector(shape->getPosition().x, shape->getPosition().y);

    }

    void setVertices(const std::vector<Vector>& verts){

        vertices = verts;

        if (!isBox && vertices.size() >= 3){

            Vector centroid = computePolygonCentroid(vertices);

            if(std::isfinite(centroid.x) && std::isfinite(centroid.y)){
                for(Vector& v : vertices){
                    v = v - centroid;
                }
                position = position + centroid;
                if(shape){
                    shape->setPosition(position.x, position.y);
                }
            }

        }

        transformedVertices.resize(vertices.size());
        updateBoundsFromVertices();
        syncConvexShape();
        transformUpdateRequired = true;
        aabbUpdateRequired = true;
        updateDerivedProperties();

    }

    void setBoxDimensions(float newWidth, float newHeight){

        width = newWidth;
        height = newHeight;
        isBox = true;
        setVertices(createBoxVertices(width, height));

        if(auto rectShape = dynamic_cast<sf::RectangleShape*>(shape)){
            rectShape->setSize(sf::Vector2f(width, height));
            rectShape->setOrigin(width / 2.f, height / 2.f);
        }

    }

    bool isBoxShape() const{
        return isBox;
    }

    void updateDerivedProperties(){

        if(isStatic){
            invMass = 0.f;
            invMoI = 0.f;
        }else{
            invMass = mass > 0.f ? 1.f / mass : 0.f;
        }

        if(type == 0){

            float area = static_cast<float>(M_PI) * radius * radius;
            density = area > 0.f ? mass / area : 0.f;
            MoI = 0.5f * mass * radius * radius;

        }else if(!vertices.empty()){

            float areaTwice = 0.f;
            float inertiaNumerator = 0.f;
            size_t count = vertices.size();
            for(size_t i = 0; i < count; ++i){
                const Vector& p0 = vertices[i];
                const Vector& p1 = vertices[(i + 1) % count];
                float cross = Vector::cross(p0, p1);
                areaTwice += cross;
                float term = (p0.x * p0.x + p0.x * p1.x + p1.x * p1.x) + (p0.y * p0.y + p0.y * p1.y + p1.y * p1.y);
                inertiaNumerator += cross * term;
            }
            float area = std::fabs(areaTwice) * 0.5f;
            if(area <= 0.f){
                MoI = 0.f;
                density = 0.f;
            }else{
                density = mass / area;
                MoI = (density / 12.f) * std::fabs(inertiaNumerator);
            }

        }else{

            float area = width * height;
            density = area > 0.f ? mass / area : 0.f;
            MoI = (mass / 12.f) * (height * height + width * width);

        }

        if(isStatic || mass <= 0.f || MoI <= 0.f){
            invMoI = 0.f;
        }else{
            invMoI = 1.f / MoI;
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
    
            if(!vertices.empty()){

                std::vector<Vector> verts = getTransformedVertices();
    
                for (size_t i = 0; i < verts.size(); i++){

                    Vector v = verts[i];
    
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

private:

    void updateBoundsFromVertices(){

        if(vertices.empty()){
            width = 0.f;
            height = 0.f;
            return;
        }

        float minX = std::numeric_limits<float>::max();
        float minY = std::numeric_limits<float>::max();
        float maxX = std::numeric_limits<float>::lowest();
        float maxY = std::numeric_limits<float>::lowest();

        for(const Vector& v : vertices){
            minX = std::min(minX, v.x);
            minY = std::min(minY, v.y);
            maxX = std::max(maxX, v.x);
            maxY = std::max(maxY, v.y);
        }

        width = maxX - minX;
        height = maxY - minY;

    }

    void syncConvexShape(){

        auto convex = dynamic_cast<sf::ConvexShape*>(shape);
        if(!convex) return;

        convex->setPointCount(vertices.size());
        for(size_t i = 0; i < vertices.size(); ++i){
            convex->setPoint(i, sf::Vector2f(vertices[i].x, vertices[i].y));
        }
        convex->setOrigin(0.f, 0.f);

    }

    Vector computePolygonCentroid(const std::vector<Vector>& poly) const{

        if(poly.empty()){
            return Vector::Zero();
        }

        float crossSum = 0.f;
        Vector centroid(0.f, 0.f);
        size_t count = poly.size();

        for(size_t i = 0; i < count; ++i){
            const Vector& p0 = poly[i];
            const Vector& p1 = poly[(i + 1) % count];
            float cross = Vector::cross(p0, p1);
            crossSum += cross;
            centroid.x += (p0.x + p1.x) * cross;
            centroid.y += (p0.y + p1.y) * cross;
        }

        if(std::fabs(crossSum) < 1e-5f){
            Vector avg(0.f, 0.f);
            for(const Vector& p : poly){
                avg = avg + p;
            }
            return avg * (1.f / static_cast<float>(count));
        }

        float inv = 1.f / (3.f * crossSum);
        return centroid * inv;

    }
    

};

#endif
