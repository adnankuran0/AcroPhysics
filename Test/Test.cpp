#include <iostream>
#include <vector>
#include "../src/AcroPhysics.h"
#include <SFML/Graphics.hpp>
#include "DebugCircle.h"
#include "DebugRect.h"

int main() 
{
    acro::PhysicsWorld2D world;

    acro::RigidBody2D body(acro::Vec2(100, 100), 1.0f, false,0.5f);
    acro::RigidBody2D body2(acro::Vec2(300, 300), 1.0f, false,0.5f);

    sf::CircleShape vrt(2.0f);
	vrt.setFillColor(sf::Color::Red);
    vrt.setOrigin(1.0f, 1.0f);


	body.setCollisionShape(50.0f,50.0f);
	body2.setCollisionShape(50.0f);


	world.addBody(&body);
	world.addBody(&body2);


    DebugRect rect(body.position.x, body.position.y, body.collisionShape->getWidth(), body.collisionShape->getHeight(), sf::Color(228,177,240,255));
    DebugCircle rect2(body2.position.x, body2.position.y, body2.collisionShape->getRadius(), sf::Color(126, 96, 191, 255));

    sf::RenderWindow window(sf::VideoMode(1280, 720), "Ecro Physics Demo");
   
    sf::Clock clock;
    float deltaTime;


    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
            if (event.type == sf::Event::KeyPressed)
            {
                if (event.key.code == sf::Keyboard::Space)
                {
                    body.velocity.y = -100;
                }
            }

        }

        if (sf::Keyboard::isKeyPressed(sf::Keyboard::D))
            body.force.x = 50;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::A))
            body.force.x = -50;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::W))
            body.force.y = -50;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::S))
            body.force.y = 50;


        deltaTime = clock.restart().asSeconds();
        world.step(deltaTime);


        window.clear(sf::Color(50,50,50,255));
        rect.setPosition(body.position.x, body.position.y);
        rect2.setPosition(body2.position.x, body2.position.y);

        rect.setRotation(body.rotation);



        rect.draw(window);
        rect2.draw(window);

       
        for (int i = 0; i < 4;i++)
        {
             vrt.setPosition(body.collisionShape->transformedVertices[i].x, body.collisionShape->transformedVertices[i].y);
             window.draw(vrt);
        }

 

        window.display();
    }



    return 0; 
}
