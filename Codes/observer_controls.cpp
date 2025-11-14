
#include "World.cpp"
#include "config.hpp"
#include "Material.hpp"
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

        if(event.key.code == Keyboard::Key::Z){

            float spawnRadiusPixels = metersToPixels(0.3f);
            world.createCircle(world, spawnRadiusPixels, spawnPosition, Vector(0.f, 0.f), 0.f, 0.f, 2.f, -1.f, randomColor, false, Material::Steel);

        } 
        else if (event.key.code == Keyboard::Key::X){

            float spawnWidthPixels = metersToPixels(1.0f);
            float spawnHeightPixels = metersToPixels(0.4f);
            world.createRectangle(world, spawnWidthPixels, spawnHeightPixels, spawnPosition, Vector(0.f, 0.f), 0.f, 0.f, 4.f, -1.f, randomColor, false, Material::Steel);

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
