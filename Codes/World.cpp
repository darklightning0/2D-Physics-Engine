
#include "physics.cpp"
#include <fstream>
#include <unordered_map>
#include <stdexcept>
#include <sstream>

void World::addObject(Object* object){

    objects.push_back(object);

}

Object& World::createRectangle(World& world, float width, float height, Vector position, Vector linearVelocity, float angle, float angularVelocity, float mass, float restitution, sf::Color color, bool isStatic, Material material){
    
    RectangleShape* RectShape = new RectangleShape(Vector2f(width, height)); 
    RectShape->setOrigin(width / 2.0f, height / 2.0f);
    RectShape->setOutlineThickness(2.f);     
    RectShape->setOutlineColor(sf::Color::White);

    float resolvedRestitution = restitution < 0.f ? getMaterialRestitution(material) : restitution;

    Object* RectObject = new Object(0, width, height, position, linearVelocity, angle, angularVelocity, mass, resolvedRestitution, material, RectShape, color, isStatic, 1, true);
    world.addObject(RectObject);

    return *RectObject;

}

Object& World::createCircle(World& world, float radius, Vector position, Vector linearVelocity, float angle, float angularVelocity, float mass, float restitution, sf::Color color, bool isStatic, Material material){

    CircleShape* CircShape = new CircleShape(radius);
    CircShape->setOrigin(radius, radius);
    CircShape->setOutlineThickness(2.f);  
    CircShape->setOutlineColor(sf::Color::White);

    float resolvedRestitution = restitution < 0.f ? getMaterialRestitution(material) : restitution;

    Object* CircObject = new Object(radius, 0, 0, position, linearVelocity, angle, angularVelocity, mass, resolvedRestitution, material, CircShape, color, isStatic, 0, false);
    world.addObject(CircObject);

    return *CircObject;

}

Object& World::createPolygon(World& world, const std::vector<Vector>& localVertices, Vector position, Vector linearVelocity, float angle, float angularVelocity, float mass, float restitution, sf::Color color, bool isStatic, Material material){

    if(localVertices.size() < 3){
        throw std::invalid_argument("Polygons require at least 3 vertices");
    }

    sf::ConvexShape* polyShape = new sf::ConvexShape(localVertices.size());
    for(size_t i = 0; i < localVertices.size(); ++i){
        polyShape->setPoint(i, sf::Vector2f(localVertices[i].x, localVertices[i].y));
    }
    polyShape->setOrigin(0.f, 0.f);
    polyShape->setOutlineThickness(2.f);
    polyShape->setOutlineColor(sf::Color::White);
    polyShape->setFillColor(color);

    float resolvedRestitution = restitution < 0.f ? getMaterialRestitution(material) : restitution;

    Object* polyObject = new Object(0.f, 0.f, 0.f, position, linearVelocity, angle, angularVelocity, mass, resolvedRestitution, material, polyShape, color, isStatic, 1, false);
    polyObject->setVertices(localVertices);
    world.addObject(polyObject);

    return *polyObject;

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

SpringJoint& World::createSpringJoint(Object& objA, Object& objB, const Vector& localAnchorA, const Vector& localAnchorB, float restLength, float stiffness, float damping){

    float actualRest = restLength;
    if(actualRest < 0.f){
        Vector worldA = objA.position + rotateLocalToWorld(localAnchorA, objA.angle);
        Vector worldB = objB.position + rotateLocalToWorld(localAnchorB, objB.angle);
        actualRest = (worldB - worldA).magnitude();
    }

    SpringJoint* spring = new SpringJoint(&objA, &objB, localAnchorA, localAnchorB, actualRest, stiffness, damping);
    springJoints.push_back(spring);
    return *spring;

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

    springJoints.erase(
        std::remove_if(springJoints.begin(), springJoints.end(),
            [&](SpringJoint* spring){
                if(!spring) return true;
                if(object == nullptr) return false;
                if(spring->objA == object || spring->objB == object){
                    delete spring;
                    return true;
                }
                return false;
            }),
        springJoints.end()
    );

}

inline void World::removeObject(Object* object){

    if(!object) return;
    removeJointsForObject(object);
    auto it = std::find(objects.begin(), objects.end(), object);
    if(it != objects.end()){
        delete *it;
        objects.erase(it);
    }

}

inline void World::clear(){

    for(DistanceJoint* joint : distanceJoints){
        delete joint;
    }
    distanceJoints.clear();

    for(SpringJoint* spring : springJoints){
        delete spring;
    }
    springJoints.clear();

    for(Object* obj : objects){
        delete obj;
    }
    objects.clear();

    for(Manifold* m : contactList){
        delete m;
    }
    contactList.clear();
    indicatorList.clear();
    potentialPairs.clear();
    manifoldCache.clear();

}

void World::update(float stepTime, int iterations){

    stepTime /= iterations;

    for(int its = 0; its < iterations; its++){

        handleTransformations(stepTime, objects);


        for(Object*& obj : objects){

            handleGravity(stepTime, obj, gravity);

        }

        for(SpringJoint* spring : springJoints){
            applySpringForces(*spring, stepTime);
        }

        for(DistanceJoint* joint : distanceJoints){
            distanceJointPreStep(*joint, stepTime);
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

        springJoints.erase(
            std::remove_if(springJoints.begin(), springJoints.end(),
                [](SpringJoint* spring){
                    if(!spring) return true;
                    if(!spring->objA || !spring->objB){
                        delete spring;
                        return true;
                    }
                    return false;
                }),
            springJoints.end());


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

const std::vector<SpringJoint*>& World::getSpringJoints() const{

    return springJoints;

}

inline bool World::saveToFile(const std::string& path) const{

    std::ofstream out(path);
    if(!out.is_open()){
        return false;
    }

    std::unordered_map<Object*, size_t> indexMap;
    for(size_t i = 0; i < objects.size(); ++i){
        indexMap[objects[i]] = i;
    }

    out << "OBJECTS " << objects.size() << '\n';
    for(Object* obj : objects){
        if(!obj) continue;
        out << obj->type << ' '
            << obj->radius << ' ' << obj->width << ' ' << obj->height << ' '
            << obj->position.x << ' ' << obj->position.y << ' '
            << obj->linearVelocity.x << ' ' << obj->linearVelocity.y << ' '
            << obj->angle << ' ' << obj->angularVelocity << ' '
            << obj->mass << ' ' << obj->restitution << ' '
            << materialToString(obj->material) << ' ' << obj->isStatic << ' '
            << static_cast<int>(obj->color.r) << ' ' << static_cast<int>(obj->color.g) << ' ' << static_cast<int>(obj->color.b);

        if(obj->type == 1){
            int shapeDetail = obj->isBoxShape() ? 0 : 1;
            out << ' ' << shapeDetail;
            if(shapeDetail == 1){
                out << ' ' << obj->vertices.size();
                for(const Vector& v : obj->vertices){
                    out << ' ' << v.x << ' ' << v.y;
                }
            }
        }

        out << '\n';
    }

    out << "DISTANCE " << distanceJoints.size() << '\n';
    for(DistanceJoint* joint : distanceJoints){
        out << indexMap[joint->objA] << ' ' << indexMap[joint->objB] << ' '
            << joint->localAnchorA.x << ' ' << joint->localAnchorA.y << ' '
            << joint->localAnchorB.x << ' ' << joint->localAnchorB.y << ' '
            << joint->restLength << ' ' << joint->frequencyHz << ' ' << joint->dampingRatio << '\n';
    }

    out << "SPRINGS " << springJoints.size() << '\n';
    for(SpringJoint* spring : springJoints){
        out << indexMap[spring->objA] << ' ' << indexMap[spring->objB] << ' '
            << spring->localAnchorA.x << ' ' << spring->localAnchorA.y << ' '
            << spring->localAnchorB.x << ' ' << spring->localAnchorB.y << ' '
            << spring->restLength << ' ' << spring->stiffness << ' ' << spring->damping << '\n';
    }

    return true;

}

inline bool World::loadFromFile(const std::string& path){

    std::ifstream in(path);
    if(!in.is_open()){
        return false;
    }

    clear();

    std::string label;
    size_t count = 0;

    if(!(in >> label >> count) || label != "OBJECTS"){
        return false;
    }

    std::vector<Object*> created;
    created.reserve(count);

    std::string line;
    std::getline(in, line); // consume remainder of header line

    for(size_t i = 0; i < count; ){
        if(!std::getline(in, line)){
            return false;
        }
        if(line.empty()){
            continue;
        }

        std::istringstream iss(line);
        int type;
        float radius, width, height;
        float posX, posY, velX, velY, angle, angularVel, mass, restitution;
        std::string materialName;
        int isStaticInt;
        int colorR, colorG, colorB;

        if(!(iss >> type >> radius >> width >> height >> posX >> posY >> velX >> velY >> angle >> angularVel >> mass >> restitution >> materialName >> isStaticInt >> colorR >> colorG >> colorB)){
            return false;
        }

        int shapeDetail = 0;
        if(type == 1 && !(iss >> shapeDetail)){
            shapeDetail = 0;
        }

        size_t vertexCount = 0;
        std::vector<Vector> customVerts;
        if(type == 1 && shapeDetail == 1){
            if(!(iss >> vertexCount)){
                return false;
            }
            customVerts.resize(vertexCount);
            for(size_t v = 0; v < vertexCount; ++v){
                if(!(iss >> customVerts[v].x >> customVerts[v].y)){
                    return false;
                }
            }
        }

        bool isStatic = isStaticInt != 0;
        Material material = materialFromString(materialName);
        sf::Color color(colorR, colorG, colorB);
        Vector position(posX, posY);
        Vector velocity(velX, velY);

        Object* obj = nullptr;
        if(type == 0){
            obj = &createCircle(*this, radius, position, velocity, angle, angularVel, mass, restitution, color, isStatic, material);
        }else if(shapeDetail == 1 && customVerts.size() >= 3){
            obj = &createPolygon(*this, customVerts, position, velocity, angle, angularVel, mass, restitution, color, isStatic, material);
        }else{
            obj = &createRectangle(*this, width, height, position, velocity, angle, angularVel, mass, restitution, color, isStatic, material);
        }

        obj->MoveTo(position);
        obj->linearVelocity = velocity;
        obj->angle = angle;
        obj->angularVelocity = angularVel;
        obj->shape->setRotation(-Vector::radToAngle(angle));
        obj->transformUpdateRequired = true;
        obj->aabbUpdateRequired = true;

        created.push_back(obj);
        ++i;
    }

    if(!(in >> label) || label != "DISTANCE"){
        return false;
    }

    in >> count;
    for(size_t i = 0; i < count; ++i){
        size_t idxA, idxB;
        float ax, ay, bx, by, rest, freq, damp;
        if(!(in >> idxA >> idxB >> ax >> ay >> bx >> by >> rest >> freq >> damp)){
            return false;
        }
        if(idxA >= created.size() || idxB >= created.size()) continue;
        createDistanceJoint(*created[idxA], *created[idxB], Vector(ax, ay), Vector(bx, by), rest, freq, damp);
    }

    if(!(in >> label) || label != "SPRINGS"){
        return false;
    }

    in >> count;
    for(size_t i = 0; i < count; ++i){
        size_t idxA, idxB;
        float ax, ay, bx, by, rest, stiff, damp;
        if(!(in >> idxA >> idxB >> ax >> ay >> bx >> by >> rest >> stiff >> damp)){
            return false;
        }
        if(idxA >= created.size() || idxB >= created.size()) continue;
        createSpringJoint(*created[idxA], *created[idxB], Vector(ax, ay), Vector(bx, by), rest, stiff, damp);
    }

    return true;

}
