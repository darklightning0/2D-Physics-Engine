
#include "physics.hpp"
#include "utility.cpp"


void handleTransformtions(float deltaTime, std::vector<Object*> objects){

    for(Object*& obj : objects){

        if(obj->isStatic) continue;

        obj->linearVelocity = obj->linearVelocity + (obj->force / obj->mass) * deltaTime;
        obj->position = obj->position + obj->linearVelocity * deltaTime;
        obj->rotation = obj->rotation + obj->rotationalVelocity * deltaTime;

        obj->shape->setPosition(sf::Vector2f(obj->position.x, obj->position.y));
        obj->shape->setRotation(obj->rotation);

        obj->force = Vector(0.f, 0.f);

    }

}

void handleGravity(float deltaTime, Object* obj, Vector gravity){

    if(!obj->isStatic){

        Vector gravitationalForce = gravity * obj->mass;
        obj->force = obj->force + gravitationalForce;

    }

}


void handleCollisions(std::vector<Object*> objects, Object* obj1){

    sf::Shape* shape1 = obj1->shape;

    for(int j = 0; j < objects.size(); j++){

        if(objects[j] == obj1) continue;

        sf::Shape* shape2 = objects[j]->shape;

        sf::CircleShape* circle1 = dynamic_cast<sf::CircleShape*>(shape1);
        sf::CircleShape* circle2 = dynamic_cast<sf::CircleShape*>(shape2);

        sf::RectangleShape* rect1 = dynamic_cast<sf::RectangleShape*>(shape1);
        sf::RectangleShape* rect2 = dynamic_cast<sf::RectangleShape*>(shape2);

        if(circle1 && circle2){

            Vector normal = Vector(0.f, 0.f);
            float depth = 0.f;

            if(intersectCircles(circle1, circle2, normal, depth)){

                obj1->Move(normal * (depth / 2.f) * -1.f);
                objects[j]->Move(normal * depth / 2.f);

                resolveCollision(obj1, objects[j], normal, depth);

            }

        }else if(rect1 && rect2){

            Vector normal = Vector(0.f, 0.f);
            float depth = 0.f;

            if(intersectPolygons(getVerticles(rect1), getVerticles(rect2), normal, depth)){

                obj1->Move(normal * (depth / 2.f) * -1.f);
                objects[j]->Move(normal * depth / 2.f);

                resolveCollision(obj1, objects[j], normal, depth);

            }

        }else if((rect1 && circle2) || (rect2 && circle1)){

            Vector normal = Vector(0.f, 0.f);
            float depth = 0.f;

            sf::CircleShape* circle = circle1 ? circle1 : circle2;
            sf::RectangleShape* rect = rect1 ? rect1 : rect2;

            if(intersectCirclePolygons(center(circle), circle->getRadius(), getVerticles(rect), normal, depth)){

                obj1->Move(normal * (depth / 2.f) * -1.f);
                objects[j]->Move(normal * depth / 2.f);

                resolveCollision(obj1, objects[j], normal, depth);

            }

        }

    }

}


