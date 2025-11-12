
#include "utility.cpp"
#include <cmath>

void handleTransformations(float deltaTime, std::vector<Object*> objects){

    const float twoPi = 2.f * M_PI;

    for(Object*& obj : objects){

        if(obj->isStatic) continue;

        if(obj->angle < -twoPi) obj->angle += twoPi;
        if(obj->angle > twoPi) obj->angle -= twoPi;

        Vector acceleration = obj->force / obj->mass;
        obj->linearVelocity = obj->linearVelocity + acceleration * deltaTime;
        obj->position = obj->position + obj->linearVelocity * deltaTime;

        obj->Rotate(-obj->angularVelocity * deltaTime);

        obj->shape->setPosition(sf::Vector2f(obj->position.x, obj->position.y));

        //obj->transformUpdateRequired = true;
        //obj->aabbUpdateRequired = true;
    }
}


void handleGravity(float deltaTime, Object* obj, Vector gravity){

    obj->force = Vector(0.f, 0.f);

    if(!obj->isStatic){

        Vector gravitationalForce = gravity * obj->mass;
        obj->force = obj->force + gravitationalForce;

    }

}


void handleCollisions(std::vector<Object*>& objects, Object* obj1, std::vector<Manifold*>& contactList, std::vector<Manifold*>& indicatorList){

    for(Object* obj2 : objects){

        if(obj1 == obj2 || (obj1->isStatic && obj2->isStatic)){

            continue; 

        }

        AABB aabb1 = obj1->getAABB();
        AABB aabb2 = obj2->getAABB();

        if(!intersectAABBs(aabb1, aabb2)){

            continue;

        }

        Vector normal;
        float depth;

        if(collide(obj1, obj2, normal, depth)){

            Vector contact1;
            Vector contact2;
            int contactCount;

            findContactPoints(*obj1, *obj2, contact1, contact2, contactCount);
            contactList.push_back(new Manifold(obj1, obj2, normal, depth, contact1, contact2, contactCount));

        }

    }

}


void cleaner(std::vector<Object*>& objects, Object& obj){

    for (size_t i = 0; i < objects.size();){

        if(objects[i]->position.x < -50 || objects[i]->position.y > 900 || objects[i]->position.x > 1250){

            delete objects[i];             
            objects.erase(objects.begin() + i);  

        }else{

            ++i; 

        }
    }
}
