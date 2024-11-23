
#include "Object.hpp"
#include "utility.hpp"

Vector center(Shape* shape){

    FloatRect bounds = shape->getGlobalBounds();
    return Vector(bounds.left + bounds.width / 2.f, bounds.top + bounds.height / 2.f);

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

    Vector centerA = center(circleA);
    Vector centerB = center(circleB);

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
        Vector axis = Vector(-edge.y, edge.x);

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
        Vector axis = Vector(-edge.y, edge.x);

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

    depth /= normal.magnitude();

    if(normal.magnitude() == 0) depth = 0;
    normal.normalize();

    Vector centerA = findArithmeticMean(verticesA);
    Vector centerB = findArithmeticMean(verticesB); 

    Vector direction = centerB - centerA;

    if(direction.dot(normal) < 0.f){

        normal = -1.f * normal;

    }

    return true;

}


bool intersectCirclePolygons(Vector circleCenter, float circleRadius, std::vector<Vector> vertices, Vector& normal, float& depth){

    normal = Vector(0.f, 0.f);
    depth = std::numeric_limits<float>::max();

    Vector axis;

    float minA, maxA;
    float minB, maxB;

    for(size_t i = 0; i < vertices.size(); i++){

        Vector vA = vertices[i];
        Vector vB = vertices[(i + 1) % vertices.size()];

        Vector edge = vB - vA;
        axis = Vector(-edge.y, edge.x).normalize();

        projectVertices(vertices, axis, minA, maxA);
        projectCircle(circleCenter, circleRadius, axis, minB, maxB);

        if(minA >= maxB || minB >= maxA){

            return false;

        }

        float axisDepth = std::min(maxB - minA, maxA - minB);

        if(axisDepth < depth){

            depth = axisDepth;
            normal = axis;

        }

    }

    int cpIndex = findClosestPointOnPolygon(circleCenter, vertices);

    if (cpIndex == -1) return false;
    Vector cp = vertices[cpIndex];

    axis = cp - circleCenter;

    projectVertices(vertices, axis, minA, maxA);
    projectCircle(circleCenter, circleRadius, axis, minB, maxB);

    if(minA >= maxB || minB >= maxA){

        return false;

    }

    float axisDepth = std::min(maxB - minA, maxA - minB);

    if(axisDepth < depth){

        depth = axisDepth;
        normal = axis;

    }

    Vector polygonCenter = findArithmeticMean(vertices);
    Vector direction = polygonCenter - circleCenter;

    if(direction.dot(normal) < 0.f){

        normal = -1.f * normal;

    }

    return true;

}


void resolveCollision(Object* shape1, Object* shape2, Vector& normal, float& depth){

    Vector relativeVelocity = shape2->linearVelocity - shape1->linearVelocity;
    float e = std::min(shape1->restitution, shape2->restitution);

    float j = -(1.f + e) * relativeVelocity.dot(normal);
    j = j / ((1.f / shape1->mass) + (1.f / shape2->mass));

    shape1->linearVelocity = shape1->linearVelocity - (j / shape1->mass * normal);
    shape2->linearVelocity = shape2->linearVelocity + (j / shape1->mass * normal);



}




