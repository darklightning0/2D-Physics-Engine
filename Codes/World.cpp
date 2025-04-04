
#include "physics.cpp"

void World::addObject(Object* object){

    objects.push_back(object);

}

Object& World::createRectangle(World& world, float width, float height, Vector position, float rotation, float density, float mass, float restitution, sf::Color color, bool isStatic){

    RectangleShape* RectShape = new RectangleShape(Vector2f(width, height)); 
    RectShape->setOrigin(width / 2.0f, height / 2.0f);
    RectShape->setOutlineThickness(2.f);     
    RectShape->setOutlineColor(sf::Color::White);
    Object* RectObject = new Object(0, width, height, position, rotation, density, mass, restitution, RectShape, color, isStatic, 1);
    world.addObject(RectObject);

    return *RectObject;

}

Object& World::createCircle(World& world, float radius, Vector position, float rotation, float density, float mass, float restitution, sf::Color color, bool isStatic){

    CircleShape* CircShape = new CircleShape(radius);
    CircShape->setOrigin(radius, radius);
    CircShape->setOutlineThickness(2.f);  
    CircShape->setOutlineColor(sf::Color::White);

    Object* CircObject = new Object(radius, 0, 0, position, rotation, density, mass, restitution, CircShape, color, isStatic, 0);
    world.addObject(CircObject);

    return *CircObject;

}

void World::update(float stepTime, int iterations){

    stepTime /= iterations;

    for(int its = 0; its < iterations; its++){


        for(Object*& obj : objects){

            handleGravity(stepTime, obj, gravity);
            handleCollisions(objects, obj, contactList, indicatorList);

        }

        for(Manifold* contact : contactList){

            resolveCollision(*contact);

        }

        //cleaning

        indicatorList = contactList;

        contactList.clear();

        for(size_t i = 0; i < objects.size();){

            if(objects[i]->position.x < -50 || objects[i]->position.y > 900 || objects[i]->position.x > 1250){

                delete objects[i];            
                objects.erase(objects.begin() + i); 

            }else{

                ++i; 

            }

        }

        objects.erase(std::remove(objects.begin(), objects.end(), nullptr), objects.end());  
        handleTransformtions(stepTime, objects);

    }
        
}

float World::getGravity() const{

    return gravity.y;

}

const std::vector<Object*>& World::getObjects() const{

    return objects;

}

const std::vector<Manifold*>& World::getContacts() const{

    return contactList;

}

const std::vector<Manifold*>& World::getIndicators() const{

    return indicatorList;

}

