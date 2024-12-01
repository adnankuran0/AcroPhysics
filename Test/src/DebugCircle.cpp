#include "DebugCircle.h"

DebugCircle::DebugCircle(float posX, float posY, float radius, sf::Color color, acro::PhysicsWorld& world) : m_Radius(radius), m_Color(color)
{
	m_Shape = sf::CircleShape(m_Radius);
	m_Shape.setOrigin(m_Radius, m_Radius);
	m_Shape.setFillColor(m_Color);
	m_Shape.setPosition(posX, posY);
	m_Shape.setOutlineColor(sf::Color::White);
	m_Shape.setOutlineThickness(1.0f);
	rb = new acro::RigidBody(acro::Vec2(posX, posY), 1.0f, false, 0.5f);
	rb->setCollider(radius);
	world.addBody(rb);
}

float DebugCircle::getRadius() const
{
	return m_Radius;
}

void DebugCircle::setRadius(float radius)
{
	m_Radius = radius;
	m_Shape.setOrigin(m_Radius, m_Radius);
}

sf::Color DebugCircle::getColor() const
{
	return m_Color;
}

void DebugCircle::setColor(sf::Color color)
{
	m_Color = color;
	m_Shape.setFillColor(color);
}

void DebugCircle::setPosition(float posX, float posY)
{
	m_Shape.setPosition(posX, posY);
}

void DebugCircle::update(sf::RenderWindow& window)
{
	m_Shape.setPosition(rb->getPosition().x, rb->getPosition().y);
	window.draw(m_Shape);

}

DebugCircle::~DebugCircle()
{
	//delete rb;
}

