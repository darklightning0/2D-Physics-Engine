
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


void handleCollisions(const std::vector<std::pair<Object*, Object*>>& pairs, std::vector<Manifold*>& contactList, std::unordered_map<PairKey, CachedManifold, PairKeyHash>& manifoldCache){

    for(auto& entry : manifoldCache){
        entry.second.active = false;
    }

    for(const auto& candidate : pairs){

        Object* obj1 = candidate.first;
        Object* obj2 = candidate.second;

        if(obj1 == obj2 || (obj1->isStatic && obj2->isStatic)){
            continue;
        }

        Vector normal;
        float depth;

        if(!collide(obj1, obj2, normal, depth)){
            continue;
        }

        Vector contact1;
        Vector contact2;
        int contactCount;

        findContactPoints(*obj1, *obj2, contact1, contact2, contactCount);
        contactList.push_back(new Manifold(obj1, obj2, normal, depth, contact1, contact2, contactCount));

        PairKey key = PairKey::make(obj1, obj2);
        CachedManifold& cacheEntry = manifoldCache[key];
        cacheEntry.normal = normal;
        cacheEntry.depth = depth;
        cacheEntry.contact1 = contact1;
        cacheEntry.contact2 = contact2;
        cacheEntry.contactCount = contactCount;
        cacheEntry.framesLeft = 2;
        cacheEntry.active = true;

    }

    for(auto it = manifoldCache.begin(); it != manifoldCache.end(); ){

        if(!it->second.active){
            it->second.framesLeft--;
        }

        if(it->second.framesLeft <= 0){
            it = manifoldCache.erase(it);
        }else{
            it->second.active = false;
            ++it;
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
