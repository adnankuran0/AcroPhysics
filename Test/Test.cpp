#include <iostream>
#include <vector>
#include "../src/AcroPhysics.h"
#include <SFML/Graphics.hpp>
#include "DebugCircle.h"
#include "DebugRect.h"



int main() 
{
    acro::PhysicsWorld world;


	acro::RigidBody floor(acro::Vec2(640,710), 0.0f, true, 0.5f);
    acro::RigidBody body(acro::Vec2(100, 100), 2.0f, false,0.5f);
    acro::RigidBody body2(acro::Vec2(300, 300), 1.0f, true,0.5f);


	floor.setCollider(1280, 20);
	body.setCollider(50.0f,50.0f);
	body2.setCollider(25.0f,50.0f);

	world.addBody(&floor);
	world.addBody(&body);
	world.addBody(&body2);
    

    DebugRect rect(body.position.x, body.position.y, body.collider->getWidth(), body.collider->getHeight(), sf::Color(228,177,240,255));
    DebugRect rect2(body2.position.x, body2.position.y, body2.collider->getWidth(), body2.collider->getHeight(), sf::Color(126, 96, 191, 255));
	DebugRect floorRect(floor.position.x, floor.position.y, floor.collider->getWidth(), floor.collider->getHeight(), sf::Color(255, 255, 255, 255));

    sf::RenderWindow window(sf::VideoMode(1280, 720), "Acro Physics Demo");
   
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
            body.force.x = 30;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::A))
            body.force.x = -30;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::W))
            body.force.y = -30;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::S))
            body.force.y = 30;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Space))
            body.velocity = acro::Vec2::zero;

        deltaTime = clock.restart().asSeconds();
        world.step(deltaTime);


        window.clear(sf::Color(50,50,50,255));
        rect.setPosition(body.position.x, body.position.y);
        rect2.setPosition(body2.position.x, body2.position.y);

        //rect.setRotation(body.rotation);


        rect.draw(window);
        rect2.draw(window);
		floorRect.draw(window);


       
       


        window.display();
    }



    return 0; 
}
