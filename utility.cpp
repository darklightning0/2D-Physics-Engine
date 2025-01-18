
#include "Object.hpp"
#include "utility.hpp"

Vector change(Vector2f vector){

    return Vector(vector.x, vector.y);

}


float distance(Vector centerA, Vector centerB){

    return (centerA - centerB).magnitude();

}


Vector findArithmeticMean(std::vector<Vector> vertices){

    float sumX = 0.f;
    float sumY = 0.f;

    for(int i = 0; i < vertices.size(); i++){

        Vector v = vertices[i];
        sumX += v.x;
        sumY += v.y;

    }

    return Vector(sumX / vertices.size(), sumY / vertices.size());

}


int findClosestPointOnPolygon(Vector circleCenter, std::vector<Vector> vertices){

    int result = -1;
    float minDist = std::numeric_limits<float>::max();

    for(int i = 0; i < vertices.size(); i++){

        Vector v = vertices[i];
        float dist = distance(v, circleCenter);

        if(dist < minDist){

            minDist = dist;
            result = i;

        }

    }

    return result;

}


std::vector<Vector> getVerticles(sf::RectangleShape* rectangle){

    std::vector<Vector> vertices;
    sf::FloatRect bounds = rectangle->getGlobalBounds();

    vertices.push_back(Vector(bounds.left, bounds.top));
    vertices.push_back(Vector(bounds.left + bounds.width, bounds.top));
    vertices.push_back(Vector(bounds.left + bounds.width, bounds.top + bounds.height));
    vertices.push_back(Vector(bounds.left, bounds.top + bounds.height));

    return vertices;

}


void projectVertices(std::vector<Vector> vertices, Vector axis, float& min, float& max){

    min = std::numeric_limits<float>::max();
    max = std::numeric_limits<float>::min();

    for(int i = 0; i < vertices.size(); i++){

        Vector v = vertices[i];
        float projection = v.dot(axis);

        if(projection < min) min = projection;
        if(projection > max) max = projection;

    }

}


void projectCircle(Vector center, float radius, Vector axis, float& min, float& max){

    Vector direction = axis.normalize();
    Vector directionAndRadius = direction * radius;

    Vector p1 = center + directionAndRadius;
    Vector p2 = center - directionAndRadius;

    min = p1.dot(axis);
    max = p2.dot(axis);

    if(min > max){

        float t = min;
        min = max;
        max = t;

    }

}


bool intersectCircles(sf::CircleShape* circleA, sf::CircleShape* circleB, Vector& normal, float& depth){

    Vector centerA = change(circleA->getPosition());
    Vector centerB = change(circleB->getPosition());

    float dist = distance(centerA, centerB);
    float radii = circleA->getRadius() + circleB->getRadius();

    if(dist >= radii){

        return false;

    }

    normal = (centerB - centerA).normalize();
    depth = radii - dist;

    return true;

}


bool intersectPolygons(std::vector<Vector> verticesA, std::vector<Vector> verticesB, Vector& normal, float& depth){

    normal.zero();
    depth = std::numeric_limits<float>::max();

    for(int i = 0; i < verticesA.size(); i++){

        Vector vA = verticesA[i];
        Vector vB = verticesA[(i + 1) % verticesA.size()];

        Vector edge = vB - vA;
        Vector axis = Vector(-edge.y, edge.x).normalize();

        float minA, maxA;
        float minB, maxB;

        projectVertices(verticesA, axis, minA, maxA);
        projectVertices(verticesB, axis, minB, maxB);

        if(minA >= maxB || minB >= maxA){

            return false;

        }

        float axisDepth = std::min(maxB - minA, maxA - minB);

        if(axisDepth < depth){

            depth = axisDepth;
            normal = axis;

        }

    }

    for(int i = 0; i < verticesB.size(); i++){

        Vector vA = verticesB[i];
        Vector vB = verticesB[(i + 1) % verticesB.size()];

        Vector edge = vB - vA;
        Vector axis = Vector(-edge.y, edge.x).normalize();

        float minA, maxA;
        float minB, maxB;

        projectVertices(verticesA, axis, minA, maxA);
        projectVertices(verticesB, axis, minB, maxB);

        if(minA >= maxB || minB >= maxA){

            return false;

        }

        float axisDepth = std::min(maxB - minA, maxA - minB);

        if(axisDepth < depth){

            depth = axisDepth;
            normal = axis;

        }

    }

    Vector centerA = findArithmeticMean(verticesA);
    Vector centerB = findArithmeticMean(verticesB); 

    Vector direction = centerB - centerA;

    if(direction.dot(normal) < 0.f){

        normal = -normal;

    }
    //std::cout<<"Normal: "<<normal.x<<", "<<normal.y<<" -- Depth: "<<depth<<std::endl;

    return true;

}


bool intersectCirclePolygons(Vector circleCenter, float circleRadius, Vector polygonCenter, std::vector<Vector> vertices, Vector& normal, float& depth){

    normal = Vector(0.f, 0.f);
    depth = std::numeric_limits<float>::max();

    Vector axis = Vector(0.f, 0.f);
    float axisDepth = 0.f;

    float minA, maxA;
    float minB, maxB;

    for(int i = 0; i < vertices.size(); i++){

        Vector vA = vertices[i];
        Vector vB = vertices[(i + 1) % vertices.size()];

        Vector edge = vB - vA;
        axis = Vector(-edge.y, edge.x).normalize();

        projectVertices(vertices, axis, minA, maxA);
        projectCircle(circleCenter, circleRadius, axis, minB, maxB);

        if(minA >= maxB || minB >= maxA){

            return false;

        }

        axisDepth = std::min(maxB - minA, maxA - minB);

        if(axisDepth < depth){

            depth = axisDepth;
            normal = axis;

        }

    }

    int cpIndex = findClosestPointOnPolygon(circleCenter, vertices);

    if (cpIndex == -1) return false;
    Vector cp = vertices[cpIndex];

    axis = (cp - circleCenter).normalize();

    projectVertices(vertices, axis, minA, maxA);
    projectCircle(circleCenter, circleRadius, axis, minB, maxB);

    if(minA >= maxB || minB >= maxA){

        return false;

    }

    axisDepth = std::min(maxB - minA, maxA - minB);

    if(axisDepth < depth){

        depth = axisDepth;
        normal = axis;

    }

    Vector direction = polygonCenter - circleCenter;

    if(direction.dot(normal) < 0.f){

        normal = -normal;

    }

    return true;

}


bool intersectCirclePolygons(Vector circleCenter, float circleRadius, std::vector<Vector> vertices, Vector& normal, float& depth){

    normal = Vector(0.f, 0.f);
    depth = std::numeric_limits<float>::max();

    Vector axis = Vector(0.f, 0.f);
    float axisDepth = 0.f;

    float minA, maxA;
    float minB, maxB;

    for(int i = 0; i < vertices.size(); i++){

        Vector vA = vertices[i];
        Vector vB = vertices[(i + 1) % vertices.size()];

        Vector edge = vB - vA;
        axis = Vector(-edge.y, edge.x).normalize();

        projectVertices(vertices, axis, minA, maxA);
        projectCircle(circleCenter, circleRadius, axis, minB, maxB);

        if(minA >= maxB || minB >= maxA){

            return false;

        }

        axisDepth = std::min(maxB - minA, maxA - minB);

        if(axisDepth < depth){

            depth = axisDepth;
            normal = axis;

        }

    }

    int cpIndex = findClosestPointOnPolygon(circleCenter, vertices);

    if (cpIndex == -1) return false;
    Vector cp = vertices[cpIndex];

    axis = (cp - circleCenter).normalize();

    projectVertices(vertices, axis, minA, maxA);
    projectCircle(circleCenter, circleRadius, axis, minB, maxB);

    if(minA >= maxB || minB >= maxA){

        return false;

    }

    axisDepth = std::min(maxB - minA, maxA - minB);

    if(axisDepth < depth){

        depth = axisDepth;
        normal = axis;

    }

    Vector polygonCenter = findArithmeticMean(vertices);
    Vector direction = polygonCenter - circleCenter;

    if(direction.dot(normal) < 0.f){

        normal = -normal;

    }

    return true;

}


bool collide(Object* obj1, Object* obj2, Vector& normal, float& depth){

    normal = normal.zero();
    depth = 0.f;
    
    if(obj1->type == 1){

        if(obj2->type == 1){

            return intersectPolygons(getVerticles(static_cast<sf::RectangleShape*>(obj1->shape.get())), getVerticles(static_cast<sf::RectangleShape*>(obj2->shape.get())), normal, depth);

        }
        else if(obj2->type == 0){

            CircleShape* circle = static_cast<sf::CircleShape*>(obj2->shape.get());
            RectangleShape* rect = static_cast<sf::RectangleShape*>(obj1->shape.get());

            bool result = intersectCirclePolygons(change(circle->getPosition()), circle->getRadius(), change(rect->getPosition()), getVerticles(rect), normal, depth);

            normal = -normal;
            return result;

        }

    }
    else if(obj1->type == 0){

        if(obj2->type == 1){

            CircleShape* circle = static_cast<sf::CircleShape*>(obj1->shape.get());
            RectangleShape* rect = static_cast<sf::RectangleShape*>(obj2->shape.get());

            return intersectCirclePolygons(change(circle->getPosition()), circle->getRadius(), change(rect->getPosition()), getVerticles(rect), normal, depth);

        }
        else if(obj2->type == 0){

            return intersectCircles(static_cast<sf::CircleShape*>(obj1->shape.get()), static_cast<sf::CircleShape*>(obj2->shape.get()), normal, depth);

        }

    }

    return false;


}


void resolveCollision(Object* shape1, Object* shape2, Vector normal, float depth){

    Vector relativeVelocity = shape2->linearVelocity - shape1->linearVelocity;

    if(relativeVelocity.dot(normal) > 0){

        return;

    }

    float e = std::min(shape1->restitution, shape2->restitution);

    float j = -(1.f + e) * relativeVelocity.dot(normal);
    j = j / (shape1->getInvMass() + shape2->getInvMass());

    Vector impulse = j * normal;

    shape1->linearVelocity = shape1->linearVelocity - impulse * shape1->getInvMass();
    shape2->linearVelocity = shape2->linearVelocity + impulse * shape2->getInvMass();

}

/*
Vector getAABB(){

    float minX = 0;
    float minY = 0;
    float maxX = 0;
    float maxY = 0;

}
*/



