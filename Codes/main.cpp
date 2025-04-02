
#include "observer_controls.cpp"

using namespace std;
using namespace sf;

int main(){

    RenderWindow window(VideoMode(1200, 800), "Physics Engine");
    //window.setFramerateLimit(60);

    World world(Vector(0, 9.81));

    float cameraFactor = 10.f;


    //world.createCircle(world, 20.f, Vector(100.f, 100.f), 1.f, 1.f, 0.5, sf::Color::Red, false);
    Object* circle = &world.createCircle(world, 20.f, Vector(500.f, 100.f), 1.f, 100.f, 0.5, sf::Color::Yellow, false);

    world.createRectangle(world, 100.f, 60.f, Vector(700.f, 100.f), 1.f, 1.f, 0.5f, sf::Color::Blue, false);
    //world.createRectangle(world, 100.f, 60.f, 1.f, 1.f, 0.5f, sf::Color::Green, false);

    world.createRectangle(world, 1000.f, 60.f, Vector(600.f, 700.f), 1.f, 1.f, 0.5f, sf::Color::White, true);
    //world.createCircle(world, 100.f, Vector(600.f, 700.f), 1.f, 1.f, 0.5f, sf::Color::White, true);


    Clock clock;
    Clock chrm;

    while(window.isOpen()){

        write("   Delta Time: ", Vector(950.f, 15));
        write("    Step Time: ", Vector(950.f, 50));
        write("Object Count: ", Vector(950.f, 85));

        //pinpoint(circle);

        float stepTime = clock.restart().asSeconds() * cameraFactor;
        float chronometre = chrm.getElapsedTime().asSeconds();

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

        world.update(stepTime);

        for(auto& obj : world.getObjects()){

            if(obj->shape){

                window.draw(*obj->shape);

            }

        }

        showContacts(world.getIndicators(), window, world, true);

        printer(window);

        window.display();

    }

    return 0;
}