
#include "World.cpp"
#include "config.hpp"
#include <random>
#include <cmath>

using namespace sf;

void spawner(Event event, RenderWindow &window, World &world){

    if(event.type == Event::KeyPressed){

        Vector2i mousePosition = Mouse::getPosition(window);
        Vector spawnPosition(mousePosition.x, mousePosition.y);

        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::uniform_int_distribution<> colorDist(0, 255);   

        sf::Color randomColor(
            colorDist(gen),
            colorDist(gen),
            colorDist(gen)
        );

        if(event.key.code == Keyboard::Key::Num1){

            float spawnRadiusPixels = metersToPixels(0.3f);
            world.createCircle(world, spawnRadiusPixels, spawnPosition, Vector(0.f, 0.f), 0.f, 0.f, 2.f, 0.4f, randomColor, false, 0.6f, 0.45f);

        } 
        else if (event.key.code == Keyboard::Key::Num2){

            float spawnWidthPixels = metersToPixels(1.0f);
            float spawnHeightPixels = metersToPixels(0.4f);
            world.createRectangle(world, spawnWidthPixels, spawnHeightPixels, spawnPosition, Vector(0.f, 0.f), 0.f, 0.f, 4.f, 0.4f, randomColor, false, 0.7f, 0.5f);

        }

    }

}

float normalizeDegrees(float degrees){

    if(degrees > 360.f || degrees < -360.f){

        degrees = std::fmod(degrees, 360.f);

    }

    if(degrees > 360.f){

        degrees -= 360.f;

    }else if(degrees < -360.f){

        degrees += 360.f;

    }

    return degrees;

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

        write("m (kg): ", Vector(5, 15));
        write("pos (m): ", Vector(5, 50));
        write("teta: ", Vector(5, 85));
        write("v (m/s): ", Vector(5, 120));
        write("a (m/s^2): ", Vector(5, 155));
        write("*F: ", Vector(5, 190));
        write("*P: ", Vector(5, 225));
    
        Vector accelaration = obj->force / obj->mass;
        Vector momentum = obj->linearVelocity * obj->mass;
        Vector positionMeters = pixelsToMeters(obj->position);
        Vector velocityMeters = pixelsToMeters(obj->linearVelocity);
        Vector accelerationMeters = pixelsToMeters(accelaration);
    
        write(std::to_string(obj->mass), Vector(80, 15));
        write(std::to_string(positionMeters.x) + ", " + std::to_string(positionMeters.y), Vector(80, 50));
        float degrees = normalizeDegrees(Vector::radToAngle(obj->angle));
        write(std::to_string(degrees), Vector(80, 85));
        write(std::to_string(velocityMeters.x) + ", " + std::to_string(velocityMeters.y), Vector(80, 120));
        write(std::to_string(accelerationMeters.x) + ", " + std::to_string(accelerationMeters.y), Vector(80, 155));
        write(std::to_string(obj->force.x) + ", " + std::to_string(obj->force.y), Vector(80, 190));
        write(std::to_string(momentum.x) + ", " + std::to_string(momentum.y), Vector(80, 225));

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

void showHitboxes(RenderWindow &window, World &world, bool on){

    if(on){

        for(auto& obj : world.getObjects()){

            if(obj->shape){
        
                std::vector<Vector> verts = obj->getTransformedVertices();
    
                for(size_t i = 0; i < verts.size(); i++){
    
                    sf::CircleShape point(2.f);
                    point.setOrigin(2.f, 2.f);
                    point.setFillColor(sf::Color::Red);
                    point.setPosition(verts[i].x, verts[i].y);
                    window.draw(point);
    
                }
    
            }
    
        }

    }

}
