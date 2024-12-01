#include "DebugRect.h"

DebugRect::DebugRect(float posX, float posY, float width, float height, sf::Color color, acro::PhysicsWorld& world) : m_Width(width), m_Height(height), m_Color(color)
, m_PosX(posX), m_PosY(posY), m_Rotation(0)
{
	m_Shape = sf::RectangleShape(sf::Vector2f(m_Width, m_Height));
	m_Shape.setOrigin(m_Width / 2, m_Height / 2);
	m_Shape.setPosition(m_PosX, m_PosY);
	m_Shape.setFillColor(m_Color);
	m_Shape.setOutlineColor(sf::Color::White);
	m_Shape.setOutlineThickness(1.0f);


	rb = new acro::RigidBody(acro::Vec2(posX, posY), 1.0f, false, 0.5f);
	rb->setCollider(width, height);
	world.addBody(rb);


}

void DebugRect::setPosition(float posX, float posY)
{
	m_PosX = posX;
	m_PosY = posY;
	m_Shape.setPosition(m_PosX, m_PosY);
}

void DebugRect::setSize(float width, float height)
{
	m_Width = width;
	m_Height = height;
	m_Shape.setOrigin(m_Width / 2, m_Height / 2);
	m_Shape.setSize(sf::Vector2f(m_Width, m_Height));
}

sf::Color DebugRect::getColor() const
{
	return m_Color;
}

void DebugRect::setColor(sf::Color color)
{
	m_Color = color;
	m_Shape.setFillColor(m_Color);
}

void DebugRect::setRotation(float rotation)
{
	m_Rotation = rotation;
	m_Shape.setRotation(m_Rotation);
}

void DebugRect::update(sf::RenderWindow& window)
{
	m_Shape.setPosition(rb->getPosition().x, rb->getPosition().y);
	m_Shape.setRotation(rb->getRotation());
	window.draw(m_Shape);
}

DebugRect::~DebugRect()
{
	//TODO: Check for memory leaks
	//delete rb;
}