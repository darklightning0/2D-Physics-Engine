
#include "observer_controls.cpp"
#include "config.hpp"
#include <algorithm>

using namespace std;
using namespace sf;

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
        0.5f,
        sf::Color::Yellow,
        false,
        0.6f,
        0.4f
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
        0.5f,
        sf::Color::Blue,
        true,
        0.9f,
        0.7f
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
        0.5f,
        sf::Color::White,
        true,
        0.9f,
        0.7f
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
        0.5f,
        sf::Color::White,
        true,
        0.9f,
        0.7f
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
        0.5f,
        sf::Color::White,
        true,
        0.95f,
        0.75f
    );


    Clock clock;
    Clock chrm;

    while(window.isOpen()){

        write("   Delta Time: ", Vector(950.f, 15));
        write("    Step Time: ", Vector(950.f, 50));
        write("Object Count: ", Vector(950.f, 85));
        write("  Time Scale: ", Vector(950.f, 120));

        pinpoint(rect);

        float stepTime = clock.restart().asSeconds() * timeScale;
        float chronometre = chrm.getElapsedTime().asSeconds();

        rect->Rotate(M_PI / 4.f * stepTime); 

        write(to_string(chronometre), Vector(1100.f, 15));
        write(to_string(stepTime * 1000), Vector(1100.f, 50));
        write(to_string(world.getObjects().size()), Vector(1100.f, 85));
        write(to_string(timeScale), Vector(1100.f, 120));

        Event event;
        while (window.pollEvent(event)){
            
            if(event.type == Event::Closed){

                window.close(); 

            }

            if(event.type == Event::KeyPressed){

                if(event.key.code == Keyboard::Up){

                    timeScale = std::min(MAX_TIME_SCALE, timeScale + TIME_SCALE_STEP);

                }
                else if(event.key.code == Keyboard::Down){

                    timeScale = std::max(MIN_TIME_SCALE, timeScale - TIME_SCALE_STEP);

                }

            }

            spawner(event, window, world);
            
        }

        window.clear(Color::Black);

        world.update(stepTime, 5);

        for(auto& obj : world.getObjects()){

            if(obj->shape){

                window.draw(*obj->shape);

            }

        }

        showHitboxes(window, world, true);

        showContacts(world.getIndicators(), window, world, true);

        printer(window);

        window.display();

    }

    return 0;
}
