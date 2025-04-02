
#include "World.cpp"

using namespace sf;

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

            world.createCircle(world, 20.f, spawnPosition, 1.f, 1.f, 0.5f, randomColor, false);

        } 
        else if (event.key.code == Keyboard::Key::Num2){

            world.createRectangle(world, 100.f, 60.f, spawnPosition, 1.f, 1.f, 0.5f, randomColor, false);

        }

    }

}

std::vector<Text> texts;
Font font;

void write(String t, Vector pos, float size = 20, Color color = Color::White){

    Text text;
    text.setString(t);
    font.loadFromFile("./Assets/Fonts/worksans.ttf");
    text.setFont(font);
    text.setCharacterSize(size);
    text.setPosition(Vector2(pos.x, pos.y)); 
    text.setFillColor(Color::White);
    texts.push_back(text);

}


void printer(RenderWindow &window){

    for(auto& text : texts){

        window.draw(text);

    }

    texts.clear();

}


void pinpoint(Object* obj){

    if(!obj){

        write("Object destroyed.", Vector(5, 15));

    }else{

        write("m: ", Vector(5, 15));
        write("s: ", Vector(5, 50));
        write("v: ", Vector(5, 85));
        write("a: ", Vector(5, 120));
        write("*F: ", Vector(5, 155));
        write("*P: ", Vector(5, 190));
    
        Vector accelaration = obj->force / obj->mass;
        Vector momentum = obj->linearVelocity * obj->mass;
    
        write(std::to_string(obj->mass), Vector(40, 15));
        write(std::to_string(obj->position.x) + ", " + std::to_string(obj->position.y), Vector(40, 50));
        write(std::to_string(obj->linearVelocity.x) + ", " + std::to_string(obj->linearVelocity.y), Vector(40, 85));
        write(std::to_string(accelaration.x) + ", " + std::to_string(accelaration.y), Vector(40, 120));
        write(std::to_string(obj->force.x) + ", " + std::to_string(obj->force.y), Vector(40, 155));
        write(std::to_string(momentum.x) + ", " + std::to_string(momentum.y), Vector(40, 190));

    }



}


void createIndicator(RenderWindow &window, World& world, Vector pos){

    CircleShape* CircShape = new CircleShape(5.f); 
    CircShape->setPosition(Vector2f(pos.x, pos.y));
    CircShape->setOrigin(5, 5);
    CircShape->setOutlineThickness(3.f);     
    CircShape->setOutlineColor(Color::Red);
    CircShape->setFillColor(Color::Transparent);

    window.draw(*CircShape);

}


void showContacts(std::vector<Manifold*> indicators, RenderWindow &window, World &world, bool on){

    if(on){

        for(Manifold* ind : indicators){

            createIndicator(window, world, ind->contact1);

            if(ind->contactCount == 2){

                createIndicator(window, world, ind->contact2);

            }

        }

    }

}
