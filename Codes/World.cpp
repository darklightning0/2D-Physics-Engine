
#include "physics.cpp"

void World::addObject(Object* object){

    objects.push_back(object);

}

Object& World::createRectangle(World& world, float width, float height, Vector position, Vector linearVelocity, float angle, float angularVelocity, float mass, float restitution, sf::Color color, bool isStatic, Material material){
    
    RectangleShape* RectShape = new RectangleShape(Vector2f(width, height)); 
    RectShape->setOrigin(width / 2.0f, height / 2.0f);
    RectShape->setOutlineThickness(2.f);     
    RectShape->setOutlineColor(sf::Color::White);

    float resolvedRestitution = restitution < 0.f ? getMaterialRestitution(material) : restitution;

    Object* RectObject = new Object(0, width, height, position, linearVelocity, angle, angularVelocity, mass, resolvedRestitution, material, RectShape, color, isStatic, 1);
    world.addObject(RectObject);

    return *RectObject;

}

Object& World::createCircle(World& world, float radius, Vector position, Vector linearVelocity, float angle, float angularVelocity, float mass, float restitution, sf::Color color, bool isStatic, Material material){

    CircleShape* CircShape = new CircleShape(radius);
    CircShape->setOrigin(radius, radius);
    CircShape->setOutlineThickness(2.f);  
    CircShape->setOutlineColor(sf::Color::White);

    float resolvedRestitution = restitution < 0.f ? getMaterialRestitution(material) : restitution;

    Object* CircObject = new Object(radius, 0, 0, position, linearVelocity, angle, angularVelocity, mass, resolvedRestitution, material, CircShape, color, isStatic, 0);
    world.addObject(CircObject);

    return *CircObject;

}

DistanceJoint& World::createDistanceJoint(Object& objA, Object& objB, const Vector& localAnchorA, const Vector& localAnchorB, float restLength, float frequencyHz, float dampingRatio){

    float actualRest = restLength;
    if(actualRest < 0.f){
        Vector worldA = objA.position + rotateLocalToWorld(localAnchorA, objA.angle);
        Vector worldB = objB.position + rotateLocalToWorld(localAnchorB, objB.angle);
        actualRest = (worldB - worldA).magnitude();
    }

    DistanceJoint* joint = new DistanceJoint(&objA, &objB, localAnchorA, localAnchorB, actualRest, frequencyHz, dampingRatio);
    distanceJoints.push_back(joint);
    return *joint;

}

RevoluteJoint& World::createRevoluteJoint(Object& objA, Object& objB, const Vector& localAnchorA, const Vector& localAnchorB, float biasFactor, float softness){

    RevoluteJoint* joint = new RevoluteJoint(&objA, &objB, localAnchorA, localAnchorB, biasFactor, softness);
    revoluteJoints.push_back(joint);
    return *joint;

}

inline void World::removeJointsForObject(Object* object){

    distanceJoints.erase(
        std::remove_if(distanceJoints.begin(), distanceJoints.end(),
            [&](DistanceJoint* joint){
                if(!joint) return true;
                if(object == nullptr) return false;
                if(joint->objA == object || joint->objB == object){
                    delete joint;
                    return true;
                }
                return false;
            }),
        distanceJoints.end()
    );

    revoluteJoints.erase(
        std::remove_if(revoluteJoints.begin(), revoluteJoints.end(),
            [&](RevoluteJoint* joint){
                if(!joint) return true;
                if(object == nullptr) return false;
                if(joint->objA == object || joint->objB == object){
                    delete joint;
                    return true;
                }
                return false;
            }),
        revoluteJoints.end()
    );

}

void World::update(float stepTime, int iterations){

    stepTime /= iterations;

    for(int its = 0; its < iterations; its++){

        handleTransformations(stepTime, objects);


        for(Object*& obj : objects){

            handleGravity(stepTime, obj, gravity);

        }

        for(DistanceJoint* joint : distanceJoints){
            distanceJointPreStep(*joint, stepTime);
        }
        for(RevoluteJoint* joint : revoluteJoints){
            revoluteJointPreStep(*joint, stepTime);
        }

        potentialPairs.clear();
        generatePotentialPairs(objects, potentialPairs);
        handleCollisions(potentialPairs, contactList, manifoldCache);

        for(Manifold* contact : contactList){

            resolveCollisionWithRotation(*contact);
            positionalCorrection(*contact);

        }

        const int jointIterations = 5;
        for(int i = 0; i < jointIterations; ++i){
            for(DistanceJoint* joint : distanceJoints){
                distanceJointApplyImpulse(*joint);
            }
            for(RevoluteJoint* joint : revoluteJoints){
                revoluteJointApplyImpulse(*joint);
            }
        }

        //cleaning

        indicatorList = contactList;

        contactList.clear();

        for(size_t i = 0; i < objects.size();){

            if(objects[i]->position.x < -50 || objects[i]->position.y > 900 || objects[i]->position.x > 1250){

                removeJointsForObject(objects[i]);
                delete objects[i];            
                objects.erase(objects.begin() + i); 

            }else{

                ++i; 

            }

        }

        objects.erase(std::remove(objects.begin(), objects.end(), nullptr), objects.end());  
        distanceJoints.erase(
            std::remove_if(distanceJoints.begin(), distanceJoints.end(),
                [](DistanceJoint* joint){
                    if(!joint) return true;
                    if(!joint->objA || !joint->objB){
                        delete joint;
                        return true;
                    }
                    return false;
                }),
            distanceJoints.end());

        revoluteJoints.erase(
            std::remove_if(revoluteJoints.begin(), revoluteJoints.end(),
                [](RevoluteJoint* joint){
                    if(!joint) return true;
                    if(!joint->objA || !joint->objB){
                        delete joint;
                        return true;
                    }
                    return false;
                }),
            revoluteJoints.end());

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

const std::vector<DistanceJoint*>& World::getDistanceJoints() const{

    return distanceJoints;

}

const std::vector<RevoluteJoint*>& World::getRevoluteJoints() const{

    return revoluteJoints;

}
