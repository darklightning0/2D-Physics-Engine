
#include "World.hpp"

Vector change(Vector2f vector){

    return Vector(vector.x, vector.y);

}


float distance(Vector centerA, Vector centerB){

    return (centerA - centerB).magnitude();

}


float distanceSquared(Vector centerA, Vector centerB){

    return distance(centerA, centerB) * distance(centerA, centerB);

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


std::vector<Vector> getVertices(sf::RectangleShape* rectangle){

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
        float projection = Vector::dot(v, axis); 

        if(projection < min) min = projection;
        if(projection > max) max = projection;

    }

}


void projectCircle(Vector center, float radius, Vector axis, float& min, float& max){

    Vector direction = axis.normalize();
    Vector directionAndRadius = direction * radius;

    Vector p1 = center + directionAndRadius;
    Vector p2 = center - directionAndRadius;

    min = Vector::dot(p1, axis);
    max = Vector::dot(p2, axis);

    if(min > max){

        float t = min;
        min = max;
        max = t;

    }

}


void pointSegmentDistance(Vector point, Vector s1, Vector s2, float& distanceSq, Vector& contact){

    Vector ab = s2 - s1;
    Vector ap = point - s1;

    float proj = Vector::dot(ap, ab);  
    float abLenSq = ab.magnitudeSquared();
    float d = proj / abLenSq;
    
    if(d <= 0.f){

        contact = s1;

    }else if(d >= 1){

        contact = s2;

    }else{

        contact = s1 + ab * d;

    }

    distanceSq = distanceSquared(point, contact);

}


void findContactPoint(Vector centerA, float radiusA, Vector centerB, Vector& contactPoint){

    Vector dir = (centerB - centerA).normalize();
    contactPoint = (centerA + dir * radiusA);

}


void findContactPoint(Vector circleCenter, float circleRadius, Vector polygonCenter, std::vector<Vector> vertices, Vector& contactPoint){

    float minDistSq = std::numeric_limits<float>::max();

    for(int i = 0; i < vertices.size(); i++){

        Vector va = vertices[i];
        Vector vb = vertices[(i + 1) % vertices.size()]; 

        float distanceSq;
        Vector contact;

        pointSegmentDistance(circleCenter, va, vb, distanceSq, contact);

        if(distanceSq < minDistSq){

            minDistSq = distanceSq;
            contactPoint = contact;

        }

    }

}


void findContactPoints(Object obj1, Object obj2, Vector& contact1, Vector& contact2, int& contactCount){

    contact1 = Vector::Zero();
    contact2 = Vector::Zero();
    contactCount = 0;

    if(obj1.type == 1){

        if(obj2.type == 1){

            findContactPoints(obj1.getTransformedVertices(), obj2.getTransformedVertices(), contact1, contact2, contactCount);

        }
        else if(obj2.type == 0){

            //CircleShape* circle = static_cast<sf::CircleShape*>(obj2.shape);
            //RectangleShape* rect = static_cast<sf::RectangleShape*>(obj1.shape);
            findContactPoint(obj2.position, obj2.radius, obj1.position, obj1.getTransformedVertices(), contact1);
            contactCount = 1;

        }

    }
    else if(obj1.type == 0){

        if(obj2.type == 1){
            
            findContactPoint(obj1.position, obj1.radius, obj2.position, obj2.getTransformedVertices(), contact1);
            contactCount = 1;

        }
        else if(obj2.type == 0){

            findContactPoint(obj1.position, obj1.radius, obj2.position, contact1);
            contactCount = 1;

        }

    }

}


void findContactPoints(std::vector<Vector> verticesA, std::vector<Vector> verticesB, Vector& contact1, Vector& contact2, int& contactCount){

    contact1 = Vector::Zero();
    contact2 = Vector::Zero();
    contactCount = 0;

    float minDistSq = std::numeric_limits<float>::max();

    for(int i = 0; i < verticesA.size(); i++){

        Vector p = verticesA[i];

        for(int j = 0; j < verticesB.size(); j++){

            Vector va = verticesB[j];
            Vector vb = verticesB[(j + 1) % verticesB.size()];

            float distSq;
            Vector cp;

            pointSegmentDistance(p, va, vb, distSq, cp);

            if(Vector::nearlyEqual(distSq, minDistSq)){

                if(!Vector::nearlyEqual(cp, contact1) && !Vector::nearlyEqual(cp, contact2)){

                    contact2 = cp;
                    contactCount = 2;

                }



            }else if(distSq < minDistSq){

                minDistSq = distSq;
                contactCount = 1;
                contact1 = cp;

            }

        }

    }

    for(int i = 0; i < verticesB.size(); i++){

        Vector p = verticesB[i];

        for(int j = 0; j < verticesA.size(); j++){

            Vector va = verticesA[j];
            Vector vb = verticesA[(j + 1) % verticesA.size()];

            float distSq;
            Vector cp;

            pointSegmentDistance(p, va, vb, distSq, cp);

            if(Vector::nearlyEqual(distSq, minDistSq)){

                if(!Vector::nearlyEqual(cp, contact1) && !Vector::nearlyEqual(cp, contact2)){

                    contact2 = cp;
                    contactCount = 2;

                }



            }else if(distSq < minDistSq){

                minDistSq = distSq;
                contactCount = 1;
                contact1 = cp;

            }

        }

    }

}


bool intersectAABBs(AABB a, AABB b){

    if(a.max.x <= b.min.x || b.max.x <= a.min.x || a.max.y <= b.min.y || b.max.y <= a.min.y){

        return false;

    }

    return true;

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


bool intersectPolygons(Vector centerA, std::vector<Vector> verticesA, Vector centerB, std::vector<Vector> verticesB, Vector& normal, float& depth){

    normal.Zero();
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

    Vector direction = centerB - centerA;

    if(Vector::dot(direction, normal) < 0.f){

        normal = -normal;

    }

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

    if(Vector::dot(direction, normal) < 0.f){

        normal = -normal;

    }

    return true;

}


bool collide(Object* obj1, Object* obj2, Vector& normal, float& depth){

    normal = normal.Zero();
    depth = 0.f;
    
    if(obj1->type == 1){

        if(obj2->type == 1){

            return intersectPolygons(change(obj1->shape->getPosition()), getVertices(static_cast<sf::RectangleShape*>(obj1->shape)), change(obj2->shape->getPosition()),getVertices(static_cast<sf::RectangleShape*>(obj2->shape)), normal, depth);

        }
        else if(obj2->type == 0){

            CircleShape* circle = static_cast<sf::CircleShape*>(obj2->shape);
            RectangleShape* rect = static_cast<sf::RectangleShape*>(obj1->shape);

            bool result = intersectCirclePolygons(change(circle->getPosition()), circle->getRadius(), change(rect->getPosition()), getVertices(rect), normal, depth);

            normal = -normal;
            return result;

        }

    }
    else if(obj1->type == 0){

        if(obj2->type == 1){

            CircleShape* circle = static_cast<sf::CircleShape*>(obj1->shape);
            RectangleShape* rect = static_cast<sf::RectangleShape*>(obj2->shape);

            return intersectCirclePolygons(change(circle->getPosition()), circle->getRadius(), change(rect->getPosition()), getVertices(rect), normal, depth);

        }
        else if(obj2->type == 0){

            return intersectCircles(static_cast<sf::CircleShape*>(obj1->shape), static_cast<sf::CircleShape*>(obj2->shape), normal, depth);

        }

    }

    return false;


}


void resolveCollision( Manifold contact){

    Object* obj1 = contact.obj1;
    Object* obj2 = contact.obj2;
    Vector normal = contact.normal;
    float depth = contact.depth;

    Vector relativeVelocity = obj2->linearVelocity - obj1->linearVelocity;

    if(Vector::dot(relativeVelocity, normal) > 0){

        return;

    }

    float e = std::min(obj1->restitution, obj2->restitution);

    float j = -(1.f + e) * Vector::dot(relativeVelocity, normal);
    j = j / (obj1->invMass + obj2->invMass);

    Vector impulse = j * normal;

    obj1->force = obj1->force + Vector(0, -9.81);
    obj2->force = obj2->force + Vector(0, -9.81);

    obj1->linearVelocity = obj1->linearVelocity - impulse * obj1->invMass;
    obj2->linearVelocity = obj2->linearVelocity + impulse * obj2->invMass;

}


