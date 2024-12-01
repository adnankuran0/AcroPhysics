#pragma once
#include "SFML/Graphics.hpp"
#include "AcroPhysics.h"

class DebugRect
{
public:
	DebugRect(float posX, float posY, float width, float height,sf::Color color,acro::PhysicsWorld& world);
	acro::RigidBody* rb;

	void setPosition(float posX, float posY);
	void setSize(float width, float height);
	void setRotation(float rotation);
	sf::Color getColor() const;
	void setColor(sf::Color color);

	void update(sf::RenderWindow& window);

	~DebugRect();



private:
	float m_PosX, m_PosY;
	float m_Width;
	float m_Height;
	float m_Rotation;
	sf::Color m_Color;
	sf::RectangleShape m_Shape;
	

};