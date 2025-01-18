
#include "observer_controls.cpp"

using namespace std;
using namespace sf;

int main(){
  
    RenderWindow window(VideoMode(1200, 800), "Physics Engine");
    window.setFramerateLimit(60);

    Font font;
    font.loadFromFile("./Assets/Fonts/montserrat.ttf");
    Text text;
    text.setFont(font);
    text.setCharacterSize(50);
    text.setPosition(100, 100); 
    text.setFillColor(Color::White);

    World world(Vector(0, 9.81));

    //world.createCircle(world, 20.f, Vector(100.f, 100.f), Vector(0.f, 0.f), 0, 0, Vector(0, 0), 50.f, 1.f, sf::Color::Red, false);
    world.createCircle(world, 20.f, Vector(500.f, 100.f), Vector(0.f, 0.f), 0, 0, Vector(0, 0), 10.f, 0.5f, sf::Color::Yellow, false);

    world.createRectangle(world, 100.f, 60.f, Vector(700.f, 100.f), Vector(0.f, 0.f), 0, 0, Vector(0, 0), 10.f, 0.5f, sf::Color::Blue, false);
    //world.createRectangle(world, 100.f, 60.f, Vector(900.f, 100.f), Vector(0.f, 0.f), 0, 0, Vector(0, 0), 10.f, 0.5f, sf::Color::Green, false);

    world.createRectangle(world, 1000.f, 60.f, Vector(600.f, 700.f), Vector(0, 0), 0, 0, Vector(0, 0), 10.f, 0.5f, sf::Color::White, true);
    //world.createCircle(world, 100.f, Vector(600.f, 700.f), Vector(0, 0), 0, 0, Vector(0, 0), 10.f, 0.5f, sf::Color::White, true);

    Clock clock;

    while(window.isOpen())
    {

        float deltaTime = clock.restart().asSeconds() * world.getGravity();
        text.setString(to_string(deltaTime));

        Event event;
        while (window.pollEvent(event))
        {
            if(event.type == Event::Closed){

                window.close(); 

            }

            spawner(event, window, world);
            
        }

        window.clear(Color::Black);

        world.update(deltaTime);

        for (const auto& obj : world.getObjects()) {
            if (obj && obj->shape.get()) {
                window.draw(*obj->shape.get());
            }
        }

        window.draw(text);

        window.display();
        cout<<(world.getObjects()).size()<<endl;
        
    }

    return 0;

    
}