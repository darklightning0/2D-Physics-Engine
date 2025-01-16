
#include "World.cpp"

void spawner(Event event, RenderWindow &window, World &world){

    if(event.type == Event::KeyPressed){

        Vector2i mousePosition = Mouse::getPosition(window);
        Vector spawnPosition(mousePosition.x, mousePosition.y);

        std::srand(static_cast<unsigned int>(std::time(nullptr))); 
        sf::Color randomColor(
            std::rand() % 256, // Random red [0-255]
            std::rand() % 256, // Random green [0-255]
            std::rand() % 256  // Random blue [0-255]
        );

        if(event.key.code == Keyboard::Key::Num1){

            world.createCircle(world, 20.f, spawnPosition, Vector(0.f, 0.f), 0, 0, Vector(0, 0), 10.f, 0.5f, randomColor, false);

        } 
        else if (event.key.code == Keyboard::Key::Num2){

            world.createRectangle(world, 100.f, 60.f, spawnPosition, Vector(0.f, 0.f), 0, 0, Vector(0, 0), 10.f, 0.5f, randomColor, false);

        }

    }


}