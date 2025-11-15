
#include "observer_controls.cpp"
#include "config.hpp"
#include "Material.hpp"
#include "Joint.hpp"
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <array>
#include <random>
#include <cmath>

using namespace std;
using namespace sf;

const std::string DEFAULT_SCENE_FILE = "scene.txt";

Object* pickObjectAt(const std::vector<Object*>& objects, const sf::Vector2f& mousePosition){
    for(auto it = objects.rbegin(); it != objects.rend(); ++it){
        Object* obj = *it;
        if(!obj || !obj->shape) continue;

        sf::FloatRect bounds = obj->shape->getGlobalBounds();
        if(bounds.contains(mousePosition)){
            return obj;
        }
    }
    return nullptr;
}

std::string formatFloat(float value, int precision = 2){
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value;
    return oss.str();
}

std::string formatVector(const Vector& value, int precision = 2){
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value.x << ", " << value.y;
    return oss.str();
}

std::vector<Vector> buildRegularPolygon(int sides, float radius){
    std::vector<Vector> verts;
    if(sides < 3 || radius <= 0.f){
        return verts;
    }
    verts.reserve(sides);
    const float delta = 2.f * static_cast<float>(M_PI) / static_cast<float>(sides);
    for(int i = 0; i < sides; ++i){
        float angle = delta * i;
        verts.emplace_back(std::cos(angle) * radius, std::sin(angle) * radius);
    }
    return verts;
}

int main(){

    RenderWindow window(VideoMode(1200, 800), "Physics Engine");
    //window.setFramerateLimit(60);

    World world(Vector(0, 0));
    float timeScale = 1.f;
    std::string sceneFile = DEFAULT_SCENE_FILE;
    std::string statusMessage;
    float statusTimer = 0.f;



     //createCircle(World &world, float radius, Vector position, Vector linearVelocity, float angle, float angularVelocity, float mass, float restitution, sf::Color color, bool isStatic)
    //createRectangle(World &world, float width, float height, Vector position, Vector linearVelocity, float angle, float angularVelocity, float mass, float restitution, sf::Color color, bool isStatic)
    float circleRadiusMeters = 0.35f;
    Vector circlePositionMeters(8.0f, 1.8f);
    Object* circle = &world.createCircle(
        world,
        metersToPixels(circleRadiusMeters),
        metersToPixels(circlePositionMeters),
        metersToPixels(Vector(0.f, 0.f)),
        0.f,
        0.f,
        8.f,
        -1.f,
        sf::Color::Yellow,
        false,
        Material::Steel
    );

    float staticPlateWidthMeters = 2.5f;
    float staticPlateHeightMeters = 0.3f;
    Vector staticPlatePosMeters(11.5f, 1.2f);
    Object* rect = &world.createRectangle(
        world,
        metersToPixels(staticPlateWidthMeters),
        metersToPixels(staticPlateHeightMeters),
        metersToPixels(staticPlatePosMeters),
        metersToPixels(Vector(0.f, 0.f)),
        0.f,
        0.f,
        1.f,
        -1.f,
        sf::Color::Blue,
        true,
        Material::Steel
    );

    float downLedgeAngle = Vector::angleToRad(155.f);
    float upperLedgeAngle = Vector::angleToRad(25.f);

    world.createRectangle(
        world,
        metersToPixels(6.0f),
        metersToPixels(0.5f),
        metersToPixels(Vector(6.0f, 6.5f)),
        metersToPixels(Vector(0.f, 0.f)),
        downLedgeAngle,
        0.f,
        1.f,
        -1.f,
        sf::Color::White,
        true,
        Material::Steel
    );
    world.createRectangle(
        world,
        metersToPixels(6.0f),
        metersToPixels(0.5f),
        metersToPixels(Vector(12.5f, 3.0f)),
        metersToPixels(Vector(0.f, 0.f)),
        upperLedgeAngle,
        0.f,
        1.f,
        -1.f,
        sf::Color::White,
        true,
        Material::Steel
    );

    world.createRectangle(
        world,
        metersToPixels(16.0f),
        metersToPixels(1.0f),
        metersToPixels(Vector(10.0f, 11.5f)),
        metersToPixels(Vector(0.f, 0.f)),
        0.f,
        0.f,
        1.f,
        -1.f,
        sf::Color::Red,
        true,
        Material::Steel
    );

    std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<int> colorDist(0, 255);
    auto randomColor = [&]() -> sf::Color{
        return sf::Color(colorDist(rng), colorDist(rng), colorDist(rng));
    };

    int spawnPolygonSides = 5;
    float spawnPolygonRadius = metersToPixels(0.6f);
    const float minPolygonRadius = metersToPixels(0.15f);
    const float maxPolygonRadius = metersToPixels(4.0f);
    const float polygonRadiusStep = metersToPixels(0.1f);


    Clock clock;
    Clock chrm;

    sf::Font uiFont;
    uiFont.loadFromFile("./Assets/Fonts/worksans.ttf");

    bool paused = false;
    bool debugMode = true;
    bool stepOnce = false;
    bool uiVisible = true;
    Object* selectedObject = nullptr;
    enum class JointCreationMode { None, Distance, Spring };
    JointCreationMode jointCreationMode = JointCreationMode::None;
    Object* jointStartObject = nullptr;
    Vector jointStartLocalAnchor = Vector::Zero();
    float displayedStepTime = 0.f;
    DistanceJoint* selectedDistanceJoint = nullptr;
    SpringJoint* selectedSpringJoint = nullptr;
    bool jointAnchorDrag = false;
    SpringJoint* dragSpring = nullptr;
    DistanceJoint* dragDistance = nullptr;
    bool draggingAAnchor = false;

    const float velocityNudge = metersToPixels(1.5f);
    const float angularNudge = Vector::angleToRad(15.f);

    auto adjustLinearVelocity = [&](const Vector& delta){
        if(selectedObject && !selectedObject->isStatic){
            selectedObject->linearVelocity = selectedObject->linearVelocity + delta;
        }
    };

    auto adjustAngularVelocity = [&](float delta){
        if(selectedObject && !selectedObject->isStatic){
            selectedObject->angularVelocity += delta;
        }
    };

    auto getMouseWorld = [&]() -> Vector{
        sf::Vector2i mp = sf::Mouse::getPosition(window);
        return Vector(static_cast<float>(mp.x), static_cast<float>(mp.y));
    };

    auto refreshMassProperties = [&](Object* obj){
        if(!obj) return;
        obj->updateDerivedProperties();
    };

    auto refreshRectangleGeometry = [&](Object* obj){
        if(!obj || obj->type != 1 || !obj->isBoxShape()) return;
        obj->setBoxDimensions(obj->width, obj->height);
    };

    auto refreshCircleGeometry = [&](Object* obj){
        if(!obj || obj->type != 0) return;

        obj->transformUpdateRequired = true;
        obj->aabbUpdateRequired = true;

        if(auto circleShape = dynamic_cast<sf::CircleShape*>(obj->shape)){
            circleShape->setRadius(obj->radius);
            circleShape->setOrigin(obj->radius, obj->radius);
        }

        refreshMassProperties(obj);
    };

    auto cancelJointCreation = [&](){
        jointCreationMode = JointCreationMode::None;
        jointStartObject = nullptr;
    };

    auto setStatus = [&](const std::string& msg){
        statusMessage = msg;
        statusTimer = 3.f;
    };

    auto spawnPolygonAt = [&](const Vector& worldPos){
        auto verts = buildRegularPolygon(spawnPolygonSides, spawnPolygonRadius);
        if(verts.size() < 3){
            setStatus("Polygon needs at least 3 sides");
            return;
        }
        world.createPolygon(
            world,
            verts,
            worldPos,
            Vector::Zero(),
            0.f,
            0.f,
            4.f,
            -1.f,
            randomColor(),
            false,
            Material::Steel
        );
        setStatus(std::to_string(spawnPolygonSides) + "-gon spawned");
    };

    auto cycleMaterial = [&](Object* obj){
        static const std::array<Material, 6> order = {
            Material::Default, Material::Wood, Material::Steel,
            Material::Rubber, Material::Ice, Material::Mud
        };
        auto it = std::find(order.begin(), order.end(), obj->material);
        size_t idx = 0;
        if(it != order.end()){
            idx = (std::distance(order.begin(), it) + 1) % order.size();
        }
        obj->material = order[idx];
        setStatus(std::string("Material -> ") + materialToString(obj->material));
    };

    auto cycleDistanceJoint = [&](int direction){
        const auto& joints = world.getDistanceJoints();
        if(joints.empty()){
            selectedDistanceJoint = nullptr;
            setStatus("No distance joints");
            return;
        }
        int idx = 0;
        if(selectedDistanceJoint){
            auto it = std::find(joints.begin(), joints.end(), selectedDistanceJoint);
            if(it != joints.end()){
                idx = static_cast<int>(std::distance(joints.begin(), it)) + direction;
            }
        }else{
            idx = 0;
        }
        if(idx < 0) idx += joints.size();
        idx %= joints.size();
        selectedDistanceJoint = joints[idx];
        selectedSpringJoint = nullptr;
        selectedObject = nullptr;
        setStatus("Distance joint selected (#" + std::to_string(idx + 1) + ")");
    };

    auto cycleSpringJoint = [&](int direction){
        const auto& springs = world.getSpringJoints();
        if(springs.empty()){
            selectedSpringJoint = nullptr;
            setStatus("No springs");
            return;
        }
        int idx = 0;
        if(selectedSpringJoint){
            auto it = std::find(springs.begin(), springs.end(), selectedSpringJoint);
            if(it != springs.end()){
                idx = static_cast<int>(std::distance(springs.begin(), it)) + direction;
            }
        }else{
            idx = 0;
        }
        if(idx < 0) idx += springs.size();
        idx %= springs.size();
        selectedSpringJoint = springs[idx];
        selectedDistanceJoint = nullptr;
        selectedObject = nullptr;
        setStatus("Spring selected (#" + std::to_string(idx + 1) + ")");
    };

    enum class TextInputMode {
        None,
        SaveScene, LoadScene,
        EditMass, EditRestitution, EditWidth, EditHeight, EditRadius,
        EditColorR, EditColorG, EditColorB,
        EditDistanceRest, EditDistanceFreq, EditDistanceDamp,
        EditSpringRest, EditSpringStiffness, EditSpringDamp,
        EditPolygonSides, EditPolygonRadius
    };
    TextInputMode textInputMode = TextInputMode::None;
    bool textInputActive = false;
    std::string textInputBuffer;

    auto startTextInput = [&](TextInputMode mode, const std::string& prefill, const std::string& prompt){
        textInputMode = mode;
        textInputActive = true;
        textInputBuffer = prefill;
        setStatus(prompt);
    };

    auto commitTextInput = [&](){
        if(textInputMode == TextInputMode::SaveScene){
            std::string filename = textInputBuffer.empty() ? DEFAULT_SCENE_FILE : textInputBuffer;
            sceneFile = filename;
            if(world.saveToFile(sceneFile)){
                setStatus("Scene saved to " + sceneFile);
            }else{
                setStatus("Failed to save scene");
            }
        }else if(textInputMode == TextInputMode::LoadScene){
            std::string filename = textInputBuffer.empty() ? DEFAULT_SCENE_FILE : textInputBuffer;
            if(world.loadFromFile(filename)){
                sceneFile = filename;
                selectedObject = nullptr;
                selectedDistanceJoint = nullptr;
                selectedSpringJoint = nullptr;
                cancelJointCreation();
                setStatus("Scene loaded from " + filename);
            }else{
                setStatus("Failed to load " + filename);
            }
        }else if(textInputMode == TextInputMode::EditMass){
            if(selectedObject){
                try{
                    float newMass = std::stof(textInputBuffer);
                    if(newMass > 0.f){
                        selectedObject->mass = newMass;
                        refreshMassProperties(selectedObject);
                        setStatus("Mass updated");
                    }else{
                        setStatus("Mass must be positive");
                    }
                }catch(...){
                    setStatus("Invalid mass input");
                }
            }
        }else if(textInputMode == TextInputMode::EditRestitution){
            if(selectedObject){
                try{
                    float newRest = std::stof(textInputBuffer);
                    newRest = std::clamp(newRest, 0.f, 1.f);
                    selectedObject->restitution = newRest;
                    setStatus("Restitution updated");
                }catch(...){
                    setStatus("Invalid restitution");
                }
            }
        }else if(textInputMode == TextInputMode::EditWidth){
            if(selectedObject && selectedObject->type == 1 && selectedObject->isBoxShape()){
                try{
                    float newWidth = std::stof(textInputBuffer);
                    if(newWidth > 0.f){
                        selectedObject->width = newWidth;
                        refreshRectangleGeometry(selectedObject);
                        setStatus("Width updated");
                    }else{
                        setStatus("Width must be positive");
                    }
                }catch(...){
                    setStatus("Invalid width");
                }
            }else{
                setStatus("Select a box to edit width");
            }
        }else if(textInputMode == TextInputMode::EditHeight){
            if(selectedObject && selectedObject->type == 1 && selectedObject->isBoxShape()){
                try{
                    float newHeight = std::stof(textInputBuffer);
                    if(newHeight > 0.f){
                        selectedObject->height = newHeight;
                        refreshRectangleGeometry(selectedObject);
                        setStatus("Height updated");
                    }else{
                        setStatus("Height must be positive");
                    }
                }catch(...){
                    setStatus("Invalid height");
                }
            }else{
                setStatus("Select a box to edit height");
            }
        }else if(textInputMode == TextInputMode::EditRadius){
            if(selectedObject && selectedObject->type == 0){
                try{
                    float newRadius = std::stof(textInputBuffer);
                    if(newRadius > 0.f){
                        selectedObject->radius = newRadius;
                        refreshCircleGeometry(selectedObject);
                        setStatus("Radius updated");
                    }else{
                        setStatus("Radius must be positive");
                    }
                }catch(...){
                    setStatus("Invalid radius");
                }
            }else{
                setStatus("Select a circle to edit radius");
            }
        }else if(textInputMode == TextInputMode::EditColorR || textInputMode == TextInputMode::EditColorG || textInputMode == TextInputMode::EditColorB){
            if(selectedObject){
                try{
                    int val = std::stoi(textInputBuffer);
                    val = std::clamp(val, 0, 255);
                    if(textInputMode == TextInputMode::EditColorR) selectedObject->color.r = static_cast<sf::Uint8>(val);
                    if(textInputMode == TextInputMode::EditColorG) selectedObject->color.g = static_cast<sf::Uint8>(val);
                    if(textInputMode == TextInputMode::EditColorB) selectedObject->color.b = static_cast<sf::Uint8>(val);
                    if(selectedObject->shape) selectedObject->shape->setFillColor(selectedObject->color);
                    setStatus("Color updated");
                }catch(...){
                    setStatus("Invalid color value");
                }
            }
        }else if(textInputMode == TextInputMode::EditDistanceRest){
            if(selectedDistanceJoint){
                try{
                    float val = std::stof(textInputBuffer);
                    selectedDistanceJoint->restLength = std::max(0.f, val);
                    setStatus("Distance rest length updated");
                }catch(...){ setStatus("Invalid value"); }
            }
        }else if(textInputMode == TextInputMode::EditDistanceFreq){
            if(selectedDistanceJoint){
                try{
                    float val = std::stof(textInputBuffer);
                    selectedDistanceJoint->frequencyHz = std::max(0.f, val);
                    setStatus("Distance frequency updated");
                }catch(...){ setStatus("Invalid value"); }
            }
        }else if(textInputMode == TextInputMode::EditDistanceDamp){
            if(selectedDistanceJoint){
                try{
                    float val = std::stof(textInputBuffer);
                    selectedDistanceJoint->dampingRatio = std::max(0.f, val);
                    setStatus("Distance damping updated");
                }catch(...){ setStatus("Invalid value"); }
            }
        }else if(textInputMode == TextInputMode::EditSpringRest){
            if(selectedSpringJoint){
                try{
                    float val = std::stof(textInputBuffer);
                    selectedSpringJoint->restLength = std::max(0.f, val);
                    setStatus("Spring rest length updated");
                }catch(...){ setStatus("Invalid value"); }
            }
        }else if(textInputMode == TextInputMode::EditSpringStiffness){
            if(selectedSpringJoint){
                try{
                    float val = std::stof(textInputBuffer);
                    selectedSpringJoint->stiffness = std::max(0.f, val);
                    setStatus("Spring stiffness updated");
                }catch(...){ setStatus("Invalid value"); }
            }
        }else if(textInputMode == TextInputMode::EditSpringDamp){
            if(selectedSpringJoint){
                try{
                    float val = std::stof(textInputBuffer);
                    selectedSpringJoint->damping = std::max(0.f, val);
                    setStatus("Spring damping updated");
                }catch(...){ setStatus("Invalid value"); }
            }
        }else if(textInputMode == TextInputMode::EditPolygonSides){
            try{
                int desiredSides = std::stoi(textInputBuffer);
                desiredSides = std::clamp(desiredSides, 3, 48);
                spawnPolygonSides = desiredSides;
                setStatus("Polygon sides set to " + std::to_string(spawnPolygonSides));
            }catch(...){
                setStatus("Invalid polygon side count");
            }
        }else if(textInputMode == TextInputMode::EditPolygonRadius){
            try{
                float meters = std::stof(textInputBuffer);
                meters = std::clamp(meters, 0.1f, 5.f);
                spawnPolygonRadius = metersToPixels(meters);
                setStatus("Polygon radius set to " + formatFloat(meters) + " m");
            }catch(...){
                setStatus("Invalid polygon radius");
            }
        }

        textInputActive = false;
        textInputMode = TextInputMode::None;
        textInputBuffer.clear();
    };

    auto handlePropertyKey = [&](int digit){
        if(selectedDistanceJoint){
            switch(digit){
                case 1:
                    startTextInput(TextInputMode::EditDistanceRest, formatFloat(selectedDistanceJoint->restLength), "Edit distance rest length...");
                    break;
                case 2:
                    startTextInput(TextInputMode::EditDistanceFreq, formatFloat(selectedDistanceJoint->frequencyHz), "Edit distance frequency...");
                    break;
                case 3:
                    startTextInput(TextInputMode::EditDistanceDamp, formatFloat(selectedDistanceJoint->dampingRatio), "Edit distance damping...");
                    break;
                default:
                    setStatus("Property not available for distance joint");
                    break;
            }
            return;
        }

        if(selectedSpringJoint){
            switch(digit){
                case 1:
                    startTextInput(TextInputMode::EditSpringRest, formatFloat(selectedSpringJoint->restLength), "Edit spring rest length...");
                    break;
                case 2:
                    startTextInput(TextInputMode::EditSpringStiffness, formatFloat(selectedSpringJoint->stiffness), "Edit spring stiffness...");
                    break;
                case 3:
                    startTextInput(TextInputMode::EditSpringDamp, formatFloat(selectedSpringJoint->damping), "Edit spring damping...");
                    break;
                default:
                    setStatus("Property not available for spring");
                    break;
            }
            return;
        }

        if(!selectedObject){
            setStatus("Select an object or joint first");
            return;
        }

        switch(digit){
            case 1:
                startTextInput(TextInputMode::EditMass, formatFloat(selectedObject->mass), "Edit mass...");
                break;
            case 2:
                startTextInput(TextInputMode::EditRestitution, formatFloat(selectedObject->restitution), "Edit restitution...");
                break;
            case 3:
                cycleMaterial(selectedObject);
                break;
            case 4:
                if(selectedObject->type == 1 && selectedObject->isBoxShape()){
                    startTextInput(TextInputMode::EditWidth, formatFloat(selectedObject->width), "Edit width...");
                }else{
                    setStatus("Width applies to box primitives only");
                }
                break;
            case 5:
                if(selectedObject->type == 1 && selectedObject->isBoxShape()){
                    startTextInput(TextInputMode::EditHeight, formatFloat(selectedObject->height), "Edit height...");
                }else{
                    setStatus("Height applies to box primitives only");
                }
                break;
            case 6:
                if(selectedObject->type == 0){
                    startTextInput(TextInputMode::EditRadius, formatFloat(selectedObject->radius), "Edit radius...");
                }else{
                    setStatus("Radius applies to circles only");
                }
                break;
            case 7:
                selectedObject->isStatic = !selectedObject->isStatic;
                refreshMassProperties(selectedObject);
                setStatus(selectedObject->isStatic ? "Object set static" : "Object set dynamic");
                break;
            case 8:
                startTextInput(TextInputMode::EditColorR, std::to_string(static_cast<int>(selectedObject->color.r)), "Edit color R...");
                break;
            case 9:
                startTextInput(TextInputMode::EditColorG, std::to_string(static_cast<int>(selectedObject->color.g)), "Edit color G...");
                break;
            case 0:
                startTextInput(TextInputMode::EditColorB, std::to_string(static_cast<int>(selectedObject->color.b)), "Edit color B...");
                break;
            default:
                setStatus("No property mapped");
                break;
        }
    };

    while(window.isOpen()){

        float elapsed = clock.restart().asSeconds();
        float chronometre = chrm.getElapsedTime().asSeconds();
        float stepTime = elapsed * timeScale;

        if(statusTimer > 0.f){
            statusTimer = std::max(0.f, statusTimer - elapsed);
        }

        Event event;
        while (window.pollEvent(event)){
            if(textInputActive){
                if(event.type == Event::Closed){
                    window.close();
                    break;
                }
                if(event.type == Event::TextEntered){
                    if(event.text.unicode >= 32 && event.text.unicode < 127){
                        textInputBuffer += static_cast<char>(event.text.unicode);
                    }
                    continue;
                }
                if(event.type == Event::KeyPressed){
                    if(event.key.code == Keyboard::Enter){
                        commitTextInput();
                    }else if(event.key.code == Keyboard::Escape){
                        textInputActive = false;
                        textInputMode = TextInputMode::None;
                        textInputBuffer.clear();
                        setStatus("Input cancelled");
                    }else if(event.key.code == Keyboard::BackSpace){
                        if(!textInputBuffer.empty()) textInputBuffer.pop_back();
                    }
                }
                continue;
            }
            
            if(event.type == Event::Closed){

                window.close(); 

            }
            else if(event.type == Event::MouseButtonPressed && event.mouseButton.button == Mouse::Left){

                sf::Vector2f mousePos(static_cast<float>(event.mouseButton.x), static_cast<float>(event.mouseButton.y));

                bool jointHandleHit = false;
                auto hitJointHandle = [&](const Vector& worldAnchor, float threshold){
                    sf::Vector2f diff(mousePos.x - worldAnchor.x, mousePos.y - worldAnchor.y);
                    return (diff.x * diff.x + diff.y * diff.y) <= threshold * threshold;
                };

                float handleRadius = 8.f;
                for(DistanceJoint* joint : world.getDistanceJoints()){
                    if(!joint || !joint->objA || !joint->objB) continue;
                    Vector worldA = joint->objA->position + rotateLocalToWorld(joint->localAnchorA, joint->objA->angle);
                    Vector worldB = joint->objB->position + rotateLocalToWorld(joint->localAnchorB, joint->objB->angle);
                    if(hitJointHandle(worldA, handleRadius)){
                        selectedDistanceJoint = joint;
                        selectedSpringJoint = nullptr;
                        selectedObject = nullptr;
                        jointHandleHit = true;
                        dragDistance = joint;
                        dragSpring = nullptr;
                        draggingAAnchor = true;
                        jointAnchorDrag = true;
                        setStatus("Distance joint selected");
                        break;
                    }
                    if(hitJointHandle(worldB, handleRadius)){
                        selectedDistanceJoint = joint;
                        selectedSpringJoint = nullptr;
                        selectedObject = nullptr;
                        jointHandleHit = true;
                        dragDistance = joint;
                        dragSpring = nullptr;
                        draggingAAnchor = false;
                        jointAnchorDrag = true;
                        setStatus("Distance joint selected");
                        break;
                    }
                }

                if(!jointHandleHit){
                    for(SpringJoint* spring : world.getSpringJoints()){
                        if(!spring || !spring->objA || !spring->objB) continue;
                        Vector worldA = spring->objA->position + rotateLocalToWorld(spring->localAnchorA, spring->objA->angle);
                        Vector worldB = spring->objB->position + rotateLocalToWorld(spring->localAnchorB, spring->objB->angle);
                        if(hitJointHandle(worldA, handleRadius)){
                            selectedSpringJoint = spring;
                            selectedDistanceJoint = nullptr;
                            selectedObject = nullptr;
                            jointHandleHit = true;
                            dragSpring = spring;
                            dragDistance = nullptr;
                            draggingAAnchor = true;
                            jointAnchorDrag = true;
                            setStatus("Spring selected");
                            break;
                        }
                        if(hitJointHandle(worldB, handleRadius)){
                            selectedSpringJoint = spring;
                            selectedDistanceJoint = nullptr;
                            selectedObject = nullptr;
                            jointHandleHit = true;
                            dragSpring = spring;
                            dragDistance = nullptr;
                            draggingAAnchor = false;
                            jointAnchorDrag = true;
                            setStatus("Spring selected");
                            break;
                        }
                    }
                }

                if(jointHandleHit){
                    if(jointAnchorDrag){
                        if(dragDistance){
                            Vector local = draggingAAnchor
                                ? rotateWorldToLocal(Vector(mousePos.x, mousePos.y) - dragDistance->objA->position, dragDistance->objA->angle)
                                : rotateWorldToLocal(Vector(mousePos.x, mousePos.y) - dragDistance->objB->position, dragDistance->objB->angle);
                            if(draggingAAnchor){
                                dragDistance->localAnchorA = local;
                            }else{
                                dragDistance->localAnchorB = local;
                            }
                        }else if(dragSpring){
                            Vector local = draggingAAnchor
                                ? rotateWorldToLocal(Vector(mousePos.x, mousePos.y) - dragSpring->objA->position, dragSpring->objA->angle)
                                : rotateWorldToLocal(Vector(mousePos.x, mousePos.y) - dragSpring->objB->position, dragSpring->objB->angle);
                            if(draggingAAnchor){
                                dragSpring->localAnchorA = local;
                            }else{
                                dragSpring->localAnchorB = local;
                            }
                        }
                    }
                    continue;
                }

                if(jointCreationMode != JointCreationMode::None && jointStartObject){

                    Object* target = pickObjectAt(world.getObjects(), mousePos);
                    if(target && target != jointStartObject){

                        Vector worldPoint(mousePos.x, mousePos.y);
                        Vector localAnchorB = rotateWorldToLocal(worldPoint - target->position, target->angle);

                        Vector worldAnchorA = jointStartObject->position + rotateLocalToWorld(jointStartLocalAnchor, jointStartObject->angle);
                        Vector worldAnchorB = target->position + rotateLocalToWorld(localAnchorB, target->angle);
                        float restLength = (worldAnchorB - worldAnchorA).magnitude();

                        if(jointCreationMode == JointCreationMode::Distance){
                            world.createDistanceJoint(*jointStartObject, *target, jointStartLocalAnchor, localAnchorB, restLength);
                        }else if(jointCreationMode == JointCreationMode::Spring){
                            world.createSpringJoint(*jointStartObject, *target, jointStartLocalAnchor, localAnchorB, restLength, 20.f, 2.f);
                        }

                        selectedObject = target;

                    }

                    cancelJointCreation();

                }else{

                    selectedObject = pickObjectAt(world.getObjects(), mousePos);
                    if(selectedObject){
                        selectedDistanceJoint = nullptr;
                        selectedSpringJoint = nullptr;
                    }

                }

            }
            else if(event.type == Event::MouseButtonReleased && event.mouseButton.button == Mouse::Left){
                jointAnchorDrag = false;
                dragDistance = nullptr;
                dragSpring = nullptr;
            }
            else if(event.type == Event::KeyPressed){

                switch(event.key.code){
                    case Keyboard::Space:
                        paused = !paused;
                        break;
                    case Keyboard::N:
                        stepOnce = true;
                        break;
                    case Keyboard::Tab:
                        uiVisible = !uiVisible;
                        break;
                    case Keyboard::F3:
                        debugMode = !debugMode;
                        setStatus(std::string("Debug ") + (debugMode ? "ON" : "OFF"));
                        break;
                    case Keyboard::Down:
                        timeScale = std::max(MIN_TIME_SCALE, timeScale - TIME_SCALE_STEP);
                        break;
                    case Keyboard::Up:
                        timeScale = std::min(MAX_TIME_SCALE, timeScale + TIME_SCALE_STEP);
                        break;
                    case Keyboard::Num1:
                    case Keyboard::Numpad1:
                        handlePropertyKey(1);
                        break;
                    case Keyboard::Num2:
                    case Keyboard::Numpad2:
                        handlePropertyKey(2);
                        break;
                    case Keyboard::Num3:
                    case Keyboard::Numpad3:
                        handlePropertyKey(3);
                        break;
                    case Keyboard::Num4:
                    case Keyboard::Numpad4:
                        handlePropertyKey(4);
                        break;
                    case Keyboard::Num5:
                    case Keyboard::Numpad5:
                        handlePropertyKey(5);
                        break;
                    case Keyboard::Num6:
                    case Keyboard::Numpad6:
                        handlePropertyKey(6);
                        break;
                    case Keyboard::Num7:
                    case Keyboard::Numpad7:
                        handlePropertyKey(7);
                        break;
                    case Keyboard::Num8:
                    case Keyboard::Numpad8:
                        handlePropertyKey(8);
                        break;
                    case Keyboard::Num9:
                    case Keyboard::Numpad9:
                        handlePropertyKey(9);
                        break;
                    case Keyboard::Num0:
                    case Keyboard::Numpad0:
                        handlePropertyKey(0);
                        break;
                    case Keyboard::W:
                        adjustLinearVelocity(Vector(0.f, -velocityNudge));
                        break;
                    case Keyboard::S:
                        adjustLinearVelocity(Vector(0.f, velocityNudge));
                        break;
                    case Keyboard::A:
                        adjustLinearVelocity(Vector(-velocityNudge, 0.f));
                        break;
                    case Keyboard::D:
                        adjustLinearVelocity(Vector(velocityNudge, 0.f));
                        break;
                    case Keyboard::Q:
                        adjustAngularVelocity(-angularNudge);
                        break;
                    case Keyboard::E:
                        adjustAngularVelocity(angularNudge);
                        break;
                    case Keyboard::R:
                        if(selectedObject && !selectedObject->isStatic){
                            selectedObject->linearVelocity = Vector::Zero();
                            selectedObject->angularVelocity = 0.f;
                        }
                        break;
                    case Keyboard::P:
                        spawnPolygonAt(getMouseWorld());
                        break;
                    case Keyboard::G:
                        if(spawnPolygonSides > 3){
                            spawnPolygonSides--;
                        }
                        setStatus("Polygon sides: " + std::to_string(spawnPolygonSides));
                        break;
                    case Keyboard::H:
                        if(spawnPolygonSides < 48){
                            spawnPolygonSides++;
                        }
                        setStatus("Polygon sides: " + std::to_string(spawnPolygonSides));
                        break;
                    case Keyboard::I:
                        startTextInput(TextInputMode::EditPolygonSides, std::to_string(spawnPolygonSides), "Enter polygon sides (3-48)...");
                        break;
                    case Keyboard::Y:
                        spawnPolygonRadius = std::max(minPolygonRadius, spawnPolygonRadius - polygonRadiusStep);
                        setStatus("Polygon radius: " + formatFloat(pixelsToMeters(spawnPolygonRadius)) + " m");
                        break;
                    case Keyboard::U:
                        spawnPolygonRadius = std::min(maxPolygonRadius, spawnPolygonRadius + polygonRadiusStep);
                        setStatus("Polygon radius: " + formatFloat(pixelsToMeters(spawnPolygonRadius)) + " m");
                        break;
                    case Keyboard::O:
                        startTextInput(TextInputMode::EditPolygonRadius, formatFloat(pixelsToMeters(spawnPolygonRadius)), "Polygon radius in meters (0.15 - 4.0)...");
                        break;
                    case Keyboard::J:
                        if(selectedObject){
                            jointCreationMode = JointCreationMode::Distance;
                            jointStartObject = selectedObject;
                            Vector mouseWorld = getMouseWorld();
                            jointStartLocalAnchor = rotateWorldToLocal(mouseWorld - selectedObject->position, selectedObject->angle);
                        }
                        break;
                    case Keyboard::K:
                        if(selectedObject){
                            jointCreationMode = JointCreationMode::Spring;
                            jointStartObject = selectedObject;
                            Vector mouseWorld = getMouseWorld();
                            jointStartLocalAnchor = rotateWorldToLocal(mouseWorld - selectedObject->position, selectedObject->angle);
                        }
                        break;
                    case Keyboard::Delete:
                    case Keyboard::BackSpace:
                        if(selectedObject){
                            world.removeObject(selectedObject);
                            selectedObject = nullptr;
                            setStatus("Object removed");
                            selectedDistanceJoint = nullptr;
                            selectedSpringJoint = nullptr;
                        }
                        break;
                    case Keyboard::C:
                        world.clear();
                        selectedObject = nullptr;
                        cancelJointCreation();
                        selectedDistanceJoint = nullptr;
                        selectedSpringJoint = nullptr;
                        setStatus("Scene cleared");
                        break;
                    case Keyboard::F5:
                        startTextInput(TextInputMode::SaveScene, sceneFile, "Save scene as...");
                        break;
                    case Keyboard::F9:
                        startTextInput(TextInputMode::LoadScene, sceneFile, "Load scene...");
                        break;
                    case Keyboard::F6:
                        cycleDistanceJoint(1);
                        break;
                    case Keyboard::F7:
                        cycleSpringJoint(1);
                        break;
                    case Keyboard::Escape:
                        cancelJointCreation();
                        selectedDistanceJoint = nullptr;
                        selectedSpringJoint = nullptr;
                        break;
                    default:
                        break;
                }
            }
            else if(event.type == Event::MouseMoved && jointAnchorDrag){
                sf::Vector2f mousePos(static_cast<float>(event.mouseMove.x), static_cast<float>(event.mouseMove.y));
                if(dragDistance){
                    if(draggingAAnchor){
                        dragDistance->localAnchorA = rotateWorldToLocal(Vector(mousePos.x, mousePos.y) - dragDistance->objA->position, dragDistance->objA->angle);
                    }else{
                        dragDistance->localAnchorB = rotateWorldToLocal(Vector(mousePos.x, mousePos.y) - dragDistance->objB->position, dragDistance->objB->angle);
                    }
                }else if(dragSpring){
                    if(draggingAAnchor){
                        dragSpring->localAnchorA = rotateWorldToLocal(Vector(mousePos.x, mousePos.y) - dragSpring->objA->position, dragSpring->objA->angle);
                    }else{
                        dragSpring->localAnchorB = rotateWorldToLocal(Vector(mousePos.x, mousePos.y) - dragSpring->objB->position, dragSpring->objB->angle);
                    }
                }
            }

            spawner(event, window, world);
            
        }

        bool shouldSimulate = !paused || stepOnce;
        if(shouldSimulate){
            world.update(stepTime, 5);
            stepOnce = false;
            displayedStepTime = stepTime;
        }else{
            displayedStepTime = 0.f;
        }

        const auto& objects = world.getObjects();
        if(selectedObject){
            auto it = std::find(objects.begin(), objects.end(), selectedObject);
            if(it == objects.end()){
                selectedObject = nullptr;
            }else if(selectedObject->type == 1){
                selectedObject->transformUpdateRequired = true;
                selectedObject->aabbUpdateRequired = true;
            }
        }

        const auto& distanceList = world.getDistanceJoints();
        if(selectedDistanceJoint){
            auto it = std::find(distanceList.begin(), distanceList.end(), selectedDistanceJoint);
            if(it == distanceList.end()){
                selectedDistanceJoint = nullptr;
            }
        }

        const auto& springList = world.getSpringJoints();
        if(selectedSpringJoint){
            auto it = std::find(springList.begin(), springList.end(), selectedSpringJoint);
            if(it == springList.end()){
                selectedSpringJoint = nullptr;
            }
        }

        window.clear(Color::Black);

        for(auto& obj : objects){

            if(obj->shape){

                window.draw(*obj->shape);

                if(obj->type == 0){

                    sf::CircleShape* circleShape = dynamic_cast<sf::CircleShape*>(obj->shape);
                    if(circleShape){
                        sf::Vertex line[] = {
                            sf::Vertex(circleShape->getPosition(), sf::Color::Red),
                            sf::Vertex(circleShape->getPosition() + sf::Vector2f(std::cos(obj->angle), -std::sin(obj->angle)) * circleShape->getRadius(), sf::Color::Red)
                        };
                        window.draw(line, 2, sf::Lines);
                    }

                }

            }

        }

        if(selectedObject && selectedObject->shape){
            sf::CircleShape marker(8.f);
            marker.setOrigin(8.f, 8.f);
            marker.setPosition(selectedObject->position.x, selectedObject->position.y);
            marker.setFillColor(sf::Color::Transparent);
            marker.setOutlineThickness(2.f);
            marker.setOutlineColor(sf::Color::Yellow);
            window.draw(marker);
        }

        for(const DistanceJoint* joint : world.getDistanceJoints()){
            if(!joint || !joint->objA || !joint->objB) continue;
            Vector worldA = joint->objA->position + rotateLocalToWorld(joint->localAnchorA, joint->objA->angle);
            Vector worldB = joint->objB->position + rotateLocalToWorld(joint->localAnchorB, joint->objB->angle);
            sf::Color color = (joint == selectedDistanceJoint) ? sf::Color::White : sf::Color::Cyan;
            sf::Vertex line[] = {
                sf::Vertex(sf::Vector2f(worldA.x, worldA.y), color),
                sf::Vertex(sf::Vector2f(worldB.x, worldB.y), color)
            };
            window.draw(line, 2, sf::Lines);

            sf::CircleShape handle(4.f);
            handle.setOrigin(4.f, 4.f);
            handle.setFillColor(color);
            handle.setPosition(worldA.x, worldA.y);
            window.draw(handle);
            handle.setPosition(worldB.x, worldB.y);
            window.draw(handle);
        }

        for(const SpringJoint* spring : world.getSpringJoints()){
            if(!spring || !spring->objA || !spring->objB) continue;
            Vector worldA = spring->objA->position + rotateLocalToWorld(spring->localAnchorA, spring->objA->angle);
            Vector worldB = spring->objB->position + rotateLocalToWorld(spring->localAnchorB, spring->objB->angle);
            sf::Color color = (spring == selectedSpringJoint) ? sf::Color::White : sf::Color::Green;
            sf::Vertex line[] = {
                sf::Vertex(sf::Vector2f(worldA.x, worldA.y), color),
                sf::Vertex(sf::Vector2f(worldB.x, worldB.y), color)
            };
            window.draw(line, 2, sf::Lines);

            sf::CircleShape handle(4.f);
            handle.setOrigin(4.f, 4.f);
            handle.setFillColor(color);
            handle.setPosition(worldA.x, worldA.y);
            window.draw(handle);
            handle.setPosition(worldB.x, worldB.y);
            window.draw(handle);
        }

        if(jointCreationMode != JointCreationMode::None && jointStartObject){
            Vector start = jointStartObject->position + rotateLocalToWorld(jointStartLocalAnchor, jointStartObject->angle);
            Vector mouseWorld = getMouseWorld();
            sf::Color color = (jointCreationMode == JointCreationMode::Spring) ? sf::Color::Green : sf::Color::Yellow;
            sf::Vertex line[] = {
                sf::Vertex(sf::Vector2f(start.x, start.y), color),
                sf::Vertex(sf::Vector2f(mouseWorld.x, mouseWorld.y), color)
            };
            window.draw(line, 2, sf::Lines);
        }

        if(debugMode){
            showHitboxes(window, world, true);
            showContacts(world.getIndicators(), window, world, true);

            for(auto& obj : objects){
                if(!obj) continue;
                Vector vel = obj->linearVelocity;
                sf::Vertex velocityLine[] = {
                    sf::Vertex(sf::Vector2f(obj->position.x, obj->position.y), sf::Color::Blue),
                    sf::Vertex(sf::Vector2f(obj->position.x + vel.x * 0.2f, obj->position.y + vel.y * 0.2f), sf::Color::Blue)
                };
                window.draw(velocityLine, 2, sf::Lines);

                sf::Text velText("v:" + formatVector(pixelsToMeters(obj->linearVelocity)), uiFont, 12);
                velText.setFillColor(sf::Color::White);
                velText.setPosition(obj->position.x + 5.f, obj->position.y - 20.f);
                window.draw(velText);
            }
        }

        if(uiVisible){
            const float panelWidth = 340.f;
            float panelHeight = 320.f;
            sf::Vector2f panelPos(20.f, 20.f);

            sf::RectangleShape panel(sf::Vector2f(panelWidth, panelHeight));
            panel.setPosition(panelPos);
            panel.setFillColor(sf::Color(0, 0, 0, 170));
            panel.setOutlineColor(sf::Color(80, 80, 80, 200));
            panel.setOutlineThickness(1.5f);
            window.draw(panel);

            float lineY = panelPos.y + 12.f;
            auto drawLine = [&](const std::string& text){
                sf::Text line(text, uiFont, 16);
                line.setFillColor(sf::Color::White);
                line.setPosition(panelPos.x + 12.f, lineY);
                window.draw(line);
                lineY += 20.f;
            };

            drawLine("Status: " + std::string(paused ? "Paused" : "Running"));
            drawLine("Delta (ms): " + formatFloat(elapsed * 1000.f));
            drawLine("Step (ms): " + formatFloat(displayedStepTime * 1000.f));
            drawLine("Time Scale: " + formatFloat(timeScale));
            drawLine("Objects: " + std::to_string(objects.size()));
            drawLine("Joints: " + std::to_string(world.getDistanceJoints().size()));
            drawLine("Springs: " + std::to_string(world.getSpringJoints().size()));
            drawLine("Scene file: " + sceneFile);
            drawLine("Polygon radius: " + formatFloat(pixelsToMeters(spawnPolygonRadius)) + " m");
            lineY += 6.f;
            drawLine("Simulation Controls:");
            drawLine(" Space - Pause/Play");
            drawLine(" N - Step once");
            drawLine(" Up/Down - Time scale +/-");
            drawLine(" F5 Save / F9 Load");
            drawLine(" C - Clear scene");
            drawLine(" Mouse L - Select body");
            drawLine(" Z/X - Spawn circle/box");
            drawLine(" P - Spawn polygon (" + std::to_string(spawnPolygonSides) + " sides)");
            drawLine(" G/H - Polygon sides -/+");
            drawLine(" I - Set polygon sides");
            drawLine(" Y/U - Polygon radius -/+");
            drawLine(" O - Set polygon radius");
            drawLine(" Delete/Backspace - Remove selected");
            drawLine(" F3 - Toggle debug visuals");

            // Move creation/joint info to right panel later

        if(jointCreationMode != JointCreationMode::None){
            lineY += 6.f;
            drawLine(std::string(jointCreationMode == JointCreationMode::Spring ? "Spring" : "Distance") + " joint: click target body...");
        }

        if(statusTimer > 0.f){
            lineY += 6.f;
            drawLine("Status: " + statusMessage);
        }

        if(textInputActive){
            lineY += 6.f;
            std::string modeLabel;
            switch(textInputMode){
                case TextInputMode::SaveScene: modeLabel = "Save"; break;
                case TextInputMode::LoadScene: modeLabel = "Load"; break;
                case TextInputMode::EditMass: modeLabel = "Mass"; break;
                case TextInputMode::EditRestitution: modeLabel = "Restitution"; break;
                case TextInputMode::EditWidth: modeLabel = "Width"; break;
                case TextInputMode::EditHeight: modeLabel = "Height"; break;
                case TextInputMode::EditRadius: modeLabel = "Radius"; break;
                case TextInputMode::EditColorR: modeLabel = "Color R"; break;
                case TextInputMode::EditColorG: modeLabel = "Color G"; break;
                case TextInputMode::EditColorB: modeLabel = "Color B"; break;
                case TextInputMode::EditDistanceRest: modeLabel = "Joint Rest"; break;
                case TextInputMode::EditDistanceFreq: modeLabel = "Joint Freq"; break;
                case TextInputMode::EditDistanceDamp: modeLabel = "Joint Damp"; break;
                case TextInputMode::EditSpringRest: modeLabel = "Spring Rest"; break;
                case TextInputMode::EditSpringStiffness: modeLabel = "Spring Stiff"; break;
                case TextInputMode::EditSpringDamp: modeLabel = "Spring Damp"; break;
                case TextInputMode::EditPolygonSides: modeLabel = "Polygon sides"; break;
                case TextInputMode::EditPolygonRadius: modeLabel = "Polygon radius"; break;
                default: modeLabel = ""; break;
            }
            drawLine("Input (" + modeLabel + "): " + textInputBuffer + "_");
        }

        const float rightPanelWidth = 360.f;
        sf::Vector2f rightPanelPos(window.getSize().x - rightPanelWidth - 20.f, 20.f);
        float rightHeight = 220.f;
        if(selectedObject) rightHeight += 220.f;
        if(selectedDistanceJoint || selectedSpringJoint) rightHeight += 120.f;

        sf::RectangleShape rightPanel(sf::Vector2f(rightPanelWidth, rightHeight));
        rightPanel.setPosition(rightPanelPos);
        rightPanel.setFillColor(sf::Color(0, 0, 0, 170));
        rightPanel.setOutlineColor(sf::Color(80, 80, 80, 200));
        rightPanel.setOutlineThickness(1.5f);
        window.draw(rightPanel);

        float rightY = rightPanelPos.y + 12.f;
        auto drawRight = [&](const std::string& text){
            sf::Text line(text, uiFont, 16);
            line.setFillColor(sf::Color::White);
            line.setPosition(rightPanelPos.x + 12.f, rightY);
            window.draw(line);
            rightY += 20.f;
        };

        drawRight("Selection:");
        if(selectedObject){
            drawRight(" Body selected");
        }else if(selectedDistanceJoint){
            drawRight(" Distance joint selected");
        }else if(selectedSpringJoint){
            drawRight(" Spring selected");
        }else{
            drawRight(" None selected");
        }

        if(selectedObject){
            rightY += 6.f;
            drawRight("Body Props (numbers map):");
            Vector posMeters = pixelsToMeters(selectedObject->position);
            Vector velMeters = pixelsToMeters(selectedObject->linearVelocity);
            drawRight(" [1] Mass: " + formatFloat(selectedObject->mass));
            drawRight(std::string(" [3] Material: ") + materialToString(selectedObject->material));
            drawRight(" Pos: " + formatVector(posMeters));
            drawRight(" Vel: " + formatVector(velMeters));
            drawRight(" [2] Restitution: " + formatFloat(selectedObject->restitution));
            drawRight(" Angle: " + formatFloat(normalizeDegrees(Vector::radToAngle(selectedObject->angle))));
            if(selectedObject->type == 1 && selectedObject->isBoxShape()){
                drawRight(" [4] Width: " + formatFloat(selectedObject->width));
                drawRight(" [5] Height: " + formatFloat(selectedObject->height));
            }else{
                drawRight(" [4] Width: -");
                drawRight(" [5] Height: -");
            }
            if(selectedObject->type == 0){
                drawRight(" [6] Radius: " + formatFloat(selectedObject->radius));
            }else{
                drawRight(" [6] Radius: -");
            }
            if(selectedObject->type == 1){
                drawRight(" Vertices: " + std::to_string(selectedObject->vertices.size()));
                if(!selectedObject->isBoxShape()){
                    drawRight(" (custom polygon)");
                }
            }
            drawRight(std::string(" [7] Static: ") + (selectedObject->isStatic ? "Yes" : "No"));
            drawRight(" [8] Color R: " + std::to_string(selectedObject->color.r));
            drawRight(" [9] Color G: " + std::to_string(selectedObject->color.g));
            drawRight(" [0] Color B: " + std::to_string(selectedObject->color.b));
        }

        if(selectedDistanceJoint){
            rightY += 6.f;
            drawRight("Distance Joint (F6 cycle):");
            drawRight(" [1] Rest length: " + formatFloat(selectedDistanceJoint->restLength));
            drawRight(" [2] Frequency: " + formatFloat(selectedDistanceJoint->frequencyHz));
            drawRight(" [3] Damping ratio: " + formatFloat(selectedDistanceJoint->dampingRatio));
        }

        if(selectedSpringJoint){
            rightY += 6.f;
            drawRight("Spring (F7 cycle):");
            drawRight(" [1] Rest length: " + formatFloat(selectedSpringJoint->restLength));
            drawRight(" [2] Stiffness: " + formatFloat(selectedSpringJoint->stiffness));
            drawRight(" [3] Damping: " + formatFloat(selectedSpringJoint->damping));
        }
        }

        window.display();

    }

    return 0;
}
