#include <iostream>
#include <vector>
#include "AcroPhysics.h"
#include <SFML/Graphics.hpp>
#include "DebugCircle.h"
#include "DebugRect.h"


//TODO: optimizations
//TODO: Narrow and broad phase
//TODO: Resolving with rotation and friction
//TODO: Attaching metarials to bodies
//TODO: Events
//TODO: Areas

int main() 
{
    acro::PhysicsWorld world;

    acro::RigidBody body(acro::Vec2(100, 100), 2.0f, false,0.5f);
    acro::RigidBody floor(acro::Vec2(640, 710), 1.0f,true,0.5f);
    acro::RigidBody wall1(acro::Vec2(5, 360), 1.0f,true,0.5f);
    acro::RigidBody wall2(acro::Vec2(1275,360), 1.0f,true,0.5f);

	body.setCollider(50.0f,50.0f);
	floor.setCollider(1280, 10);
    wall1.setCollider(10, 720);
    wall2.setCollider(10, 720);

	std::vector<DebugCircle> circles;
	std::vector<DebugRect> rects;

	sf::CircleShape circle(4);
	circle.setFillColor(sf::Color::White);
	circle.setOrigin(2.0f, 2.0f);


	world.addBody(&floor);
	world.addBody(&body);
	world.addBody(&wall1);
	world.addBody(&wall2);

	sf::RectangleShape bodyShape(sf::Vector2f(50, 50));

	bodyShape.setFillColor(sf::Color::Red);
	bodyShape.setOutlineColor(sf::Color::White);
	bodyShape.setOrigin(25, 25);



    sf::RenderWindow window(sf::VideoMode(1280, 720), "Acro Physics Demo");
    window.setSize(sf::Vector2u(1280, 720));
    sf::Clock clock;
    float deltaTime;

    bool debugMode = false;


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
                    body.setVelocity(body.getVelocity().x, -100);
                }
                if (event.key.code == sf::Keyboard::BackSpace)
                {
                    debugMode = !debugMode;
                }

            }

            if (event.type == sf::Event::MouseButtonPressed)
            {
                if (event.mouseButton.button == sf::Mouse::Button::Left)
                {
					circles.emplace_back(event.mouseButton.x, event.mouseButton.y, 20, sf::Color(142,25,60,255), world);
                }
				if (event.mouseButton.button == sf::Mouse::Button::Right)
				{
					rects.emplace_back(event.mouseButton.x, event.mouseButton.y, 40, 40, sf::Color(25, 65, 142, 255), world);
                }
            }

        }

        if (sf::Keyboard::isKeyPressed(sf::Keyboard::D))
            body.applyForce(30,0);
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::A))
            body.applyForce(-30, 0);
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::W))
            body.applyForce(0, -30);
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::S))
            body.applyForce(0, 30);
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Space))
            body.setVelocity(0,0);

        deltaTime = clock.restart().asSeconds();
        world.step(deltaTime,4);


        window.clear(sf::Color(50,50,50,255));


		for (auto& circle : circles)
        {
			circle.update(window);
		}

        for (auto& rect : rects)
        {
            rect.update(window);
        }

		bodyShape.setPosition(body.getPosition().x,body.getPosition().y);
		bodyShape.setRotation(body.getRotation());



		
		window.draw(bodyShape);

        /*
        
		for (auto& cp : world.getContactPoints())
        {
			circle.setFillColor(sf::Color::White);
			std::cout << cp.x << " " << cp.y << std::endl;
            circle.setPosition(cp.x, cp.y);
			window.draw(circle);
		}
        */



		if (debugMode)
            std::cout << "fps: " << 1.0f / deltaTime << " body: " << world.getBodyCount() << std::endl;

        window.display();
    }

    return 0; 
}
