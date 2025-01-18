
#include "physics.hpp"
#include "utility.cpp"


void handleTransformtions(float deltaTime, std::vector<std::shared_ptr<Object>>& objects){

    for(auto& obj : objects){

        if(obj->isStatic) continue;
        Vector acceleration = obj->force / obj->mass;
        obj->linearVelocity = obj->linearVelocity + acceleration * deltaTime;
        obj->position = obj->position + obj->linearVelocity * deltaTime;
        obj->rotation = obj->rotation + obj->rotationalVelocity * deltaTime;

        obj->shape->setPosition(sf::Vector2f(obj->position.x, obj->position.y));
        obj->shape->setRotation(obj->rotation);

        obj->force = Vector(0.f, 0.f);

    }

}

void handleGravity(float deltaTime, const std::shared_ptr<Object>& obj, Vector gravity){

    if(!obj->isStatic){

        Vector gravitationalForce = gravity * obj->mass;
        obj->force = obj->force + gravitationalForce;

    }

}


void handleCollisions(const std::vector<std::shared_ptr<Object>>& objects, std::shared_ptr<Object> obj1){

    for(const auto& obj2 : objects){

        if(obj1 == obj2 || (obj1->isStatic && obj2->isStatic)){

            continue; 

        }

        Vector normal;
        float depth;

       if(collide(obj1.get(), obj2.get(), normal, depth)){

        if(obj1->isStatic){

            obj2->Move(normal * depth);

        }
        else if(obj2->isStatic){

            obj1->Move(-normal * depth);

        }else{

            obj1->Move(-normal * depth / 2);
            obj2->Move(normal * depth / 2);

        }

        resolveCollision(obj1.get(), obj2.get(), normal, depth);

       }

        

    }

}


void cleaner(std::vector<std::shared_ptr<Object>>& objects, const std::shared_ptr<Object>& obj){

    if (obj->position.x < -50 || obj->position.y > 900 || obj->position.x > 1250){

        objects.erase(std::remove(objects.begin(), objects.end(), obj), objects.end());

    }

}


