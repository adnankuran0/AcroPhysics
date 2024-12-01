#pragma once
#include "SFML/Graphics.hpp"
#include "AcroPhysics.h"

class DebugCircle
{
public:
	acro::RigidBody* rb;
	DebugCircle(float posX, float posY, float radius, sf::Color color,acro::PhysicsWorld& world);
	float getRadius() const;
	void setRadius(float radius);
	sf::Color getColor() const;
	void setColor(sf::Color color);
	void setPosition(float posX, float posY);

	void update(sf::RenderWindow& window);

	~DebugCircle();

private:
	sf::CircleShape m_Shape;
	sf::Color m_Color;
	float m_Radius;
};