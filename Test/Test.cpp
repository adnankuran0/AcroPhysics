#include <iostream>
#include <vector>
#include "../src/AcroPhysics.h"
#include <SFML/Graphics.hpp>
#include "DebugCircle.h"
#include "DebugRect.h"

int main() 
{
    acro::PhysicsWorld2D world;

    acro::RigidBody2D body(acro::Vec2(200, 200), acro::Vec2(0, 0), 1.0f, false, 0.5);
    acro::RigidBody2D body2(acro::Vec2(0, 600), acro::Vec2(0, 0), 1.0f, true, 0.0);
    acro::RigidBody2D body3(acro::Vec2(0, 0), acro::Vec2(0, 0), 1.0f, true, 0.0);
    acro::RigidBody2D body4(acro::Vec2(1270, 0), acro::Vec2(0, 0), 1.0f, true, 0.0);
    acro::RigidBody2D body5(acro::Vec2(500, 500), acro::Vec2(0, 0), 1.0f, false, 0.0);

    body.setCollisionShape(20);
    body2.setCollisionShape(1280, 120);
    body3.setCollisionShape(10, 720);
    body4.setCollisionShape(10, 720);
    body5.setCollisionShape(20);

    world.addBody(&body);
    world.addBody(&body2);
    world.addBody(&body3);
    world.addBody(&body4);
    world.addBody(&body5);

    DebugCircle circle1(body.position.x, body.position.y, body.collisionShape->getRadius(), sf::Color::Magenta);
    DebugRect rect1(body2.position.x, body2.position.y, body2.collisionShape->getWidth(), body2.collisionShape->getHeight(), sf::Color::Blue);
    DebugRect rect2(body3.position.x, body3.position.y, body3.collisionShape->getWidth(), body3.collisionShape->getHeight(), sf::Color::Blue);
    DebugRect rect3(body4.position.x, body4.position.y, body4.collisionShape->getWidth(), body4.collisionShape->getHeight(), sf::Color::Blue);
    DebugCircle circle2(body5.position.x, body5.position.y, body5.collisionShape->getRadius(), sf::Color::Red);



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

        body.velocity = acro::Vec2(0,0);
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::D))
            body.velocity.x = 30;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::A))
            body.velocity.x = -30;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::W))
            body.velocity.y = -30;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::S))
            body.velocity.y = 30;


        deltaTime = clock.restart().asSeconds();
        world.step(deltaTime);


        window.clear();
        circle1.setPosition(body.position.x, body.position.y);
        circle2.setPosition(body5.position.x, body5.position.y);

        circle1.draw(window);
        rect1.draw(window);
        rect2.draw(window);
        rect3.draw(window);
        circle2.draw(window);
       

        window.display();
    }



    return 0; 
}
