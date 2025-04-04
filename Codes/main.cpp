
#include "observer_controls.cpp"

using namespace std;
using namespace sf;

int main(){

    RenderWindow window(VideoMode(1200, 800), "Physics Engine");
    //window.setFramerateLimit(60);

    World world(Vector(0, 9.81));

    float cameraFactor = 10.f;



    Object* circle = &world.createCircle(world, 20.f, Vector(500.f, 100.f), 0.f, 1.f, 100.f, 0.5, sf::Color::Yellow, false);

    Object* rect = &world.createRectangle(world, 150.f, 20.f, Vector(700.f, 60.f), 0.f, 1.f, 1.f, 0.5f, sf::Color::Blue, true);

    Object* downLedge = &world.createRectangle(world, 400.f, 30.f, Vector(400.f, 400.f), 155.f, 1.f, 1.f, 0.5f, sf::Color::White, true);
    Object* upperLedge = &world.createRectangle(world, 400.f, 30.f, Vector(750.f, 175.f), 25.f, 1.f, 1.f, 0.5f, sf::Color::White, true);

    world.createRectangle(world, 1000.f, 60.f, Vector(600.f, 700.f), 0.f, 1.f, 1.f, 0.5f, sf::Color::White, true);


    Clock clock;
    Clock chrm;

    while(window.isOpen()){

        write("   Delta Time: ", Vector(950.f, 15));
        write("    Step Time: ", Vector(950.f, 50));
        write("Object Count: ", Vector(950.f, 85));

        pinpoint(rect);

        float stepTime = clock.restart().asSeconds() * cameraFactor;
        float chronometre = chrm.getElapsedTime().asSeconds();

        rect->Rotate(M_PI / 2.f * stepTime * -5);

        write(to_string(chronometre), Vector(1100.f, 15));
        write(to_string(stepTime * 1000), Vector(1100.f, 50));
        write(to_string(world.getObjects().size()), Vector(1100.f, 85));

        Event event;
        while (window.pollEvent(event)){
            
            if(event.type == Event::Closed){

                window.close(); 

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