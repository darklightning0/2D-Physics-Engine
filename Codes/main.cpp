
#include "observer_controls.cpp"
#include "config.hpp"
#include "Material.hpp"
#include "Joint.hpp"
#include <algorithm>
#include <sstream>
#include <iomanip>

using namespace std;
using namespace sf;

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

std::string materialToString(Material material){
    switch(material){
        case Material::Wood: return "Wood";
        case Material::Steel: return "Steel";
        case Material::Rubber: return "Rubber";
        case Material::Ice: return "Ice";
        case Material::Mud: return "Mud";
        case Material::Default:
        default: return "Default";
    }
}

int main(){

    RenderWindow window(VideoMode(1200, 800), "Physics Engine");
    //window.setFramerateLimit(60);

    World world(Vector(0, 9.81f * PIXELS_PER_METER));
    float timeScale = 1.f;



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


    Clock clock;
    Clock chrm;

    sf::Font uiFont;
    uiFont.loadFromFile("./Assets/Fonts/worksans.ttf");

    bool paused = false;
    bool stepOnce = false;
    bool uiVisible = true;
    Object* selectedObject = nullptr;
    bool jointCreationActive = false;
    Object* jointStartObject = nullptr;
    Vector jointStartLocalAnchor = Vector::Zero();
    float displayedStepTime = 0.f;

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

    while(window.isOpen()){

        float elapsed = clock.restart().asSeconds();
        float chronometre = chrm.getElapsedTime().asSeconds();
        float stepTime = elapsed * timeScale;

        Event event;
        while (window.pollEvent(event)){
            
            if(event.type == Event::Closed){

                window.close(); 

            }
            else if(event.type == Event::MouseButtonPressed && event.mouseButton.button == Mouse::Left){

                sf::Vector2f mousePos(static_cast<float>(event.mouseButton.x), static_cast<float>(event.mouseButton.y));

                if(jointCreationActive && jointStartObject){

                    Object* target = pickObjectAt(world.getObjects(), mousePos);
                    if(target && target != jointStartObject){

                        Vector worldPoint(mousePos.x, mousePos.y);
                        Vector localAnchorB = rotateWorldToLocal(worldPoint - target->position, target->angle);
                        Vector worldAnchorA = jointStartObject->position + rotateLocalToWorld(jointStartLocalAnchor, jointStartObject->angle);
                        Vector worldAnchorB = target->position + rotateLocalToWorld(localAnchorB, target->angle);
                        float restLength = (worldAnchorB - worldAnchorA).magnitude();
                        world.createDistanceJoint(*jointStartObject, *target, jointStartLocalAnchor, localAnchorB, restLength);
                        selectedObject = target;

                    }

                    jointCreationActive = false;
                    jointStartObject = nullptr;

                }else{

                    selectedObject = pickObjectAt(world.getObjects(), mousePos);

                }

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
                    case Keyboard::Down:
                        timeScale = std::max(MIN_TIME_SCALE, timeScale - TIME_SCALE_STEP);
                        break;
                    case Keyboard::Up:
                        timeScale = std::min(MAX_TIME_SCALE, timeScale + TIME_SCALE_STEP);
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
                    case Keyboard::J:
                        if(selectedObject){
                            jointCreationActive = true;
                            jointStartObject = selectedObject;
                            Vector mouseWorld = getMouseWorld();
                            jointStartLocalAnchor = rotateWorldToLocal(mouseWorld - selectedObject->position, selectedObject->angle);
                        }
                        break;
                    case Keyboard::Escape:
                        jointCreationActive = false;
                        jointStartObject = nullptr;
                        break;
                    default:
                        break;
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
            sf::Vertex line[] = {
                sf::Vertex(sf::Vector2f(worldA.x, worldA.y), sf::Color::Cyan),
                sf::Vertex(sf::Vector2f(worldB.x, worldB.y), sf::Color::Cyan)
            };
            window.draw(line, 2, sf::Lines);
        }

        if(jointCreationActive && jointStartObject){
            Vector start = jointStartObject->position + rotateLocalToWorld(jointStartLocalAnchor, jointStartObject->angle);
            Vector mouseWorld = getMouseWorld();
            sf::Vertex line[] = {
                sf::Vertex(sf::Vector2f(start.x, start.y), sf::Color::Yellow),
                sf::Vertex(sf::Vector2f(mouseWorld.x, mouseWorld.y), sf::Color::Yellow)
            };
            window.draw(line, 2, sf::Lines);
        }

        showHitboxes(window, world, true);
        showContacts(world.getIndicators(), window, world, true);

        if(uiVisible){
            const float panelWidth = 340.f;
            float panelHeight = 260.f + (selectedObject ? 160.f : 0.f);
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
            drawLine("Selected: " + std::string(selectedObject ? materialToString(selectedObject->material) : "None"));
            lineY += 6.f;
            drawLine("Controls:");
            drawLine(" Space - Pause/Play");
            drawLine(" N - Step once");
            drawLine(" Up/Down - Time scale +/-");
            drawLine(" Mouse L - Select body");
            drawLine(" W/A/S/D - Nudge velocity");
            drawLine(" Q/E - Spin, R - Reset");
            drawLine(" 1/2 - Spawn circle/box");
            drawLine(" J - Start joint (Esc cancel)");

            if(jointCreationActive){
                lineY += 6.f;
                drawLine("Joint creation: click target body...");
            }

            if(selectedObject){
                lineY += 6.f;
                drawLine("Selected Body:");
                Vector posMeters = pixelsToMeters(selectedObject->position);
                Vector velMeters = pixelsToMeters(selectedObject->linearVelocity);
                drawLine(" Mass (kg): " + formatFloat(selectedObject->mass));
                drawLine(" Material: " + materialToString(selectedObject->material));
                drawLine(" Pos (m): " + formatVector(posMeters));
                drawLine(" Vel (m/s): " + formatVector(velMeters));
                drawLine(" Angle (deg): " + formatFloat(normalizeDegrees(Vector::radToAngle(selectedObject->angle))));
                drawLine(" Restitution: " + formatFloat(selectedObject->restitution));
            }
        }

        window.display();

    }

    return 0;
}
