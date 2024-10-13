#include "RigidBody2D.h"

namespace acro {
	RigidBody2D::RigidBody2D(Vec2 pos, float m, bool isStatic,float restitution) : position(pos), velocity(Vec2(0,0)), 
		mass(m), force(0, 0), isStatic(isStatic),restitution(restitution), rotation(0), rotationalVelocity(0){}

	RigidBody2D::RigidBody2D(const RigidBody2D& other) = default;
	
	RigidBody2D::~RigidBody2D()
	{
		delete collisionShape;
		
	}

	void RigidBody2D::applyForce(const Vec2& f)
	{
		force += f;
	}

	void RigidBody2D::applyForce(float x, float y)
	{
		force += Vec2(x,y);
	}

	void RigidBody2D::setCollisionShape(float radius)
	{
		collisionShape = new CircleShape(position.x,position.y,radius);
	}

	void RigidBody2D::setCollisionShape(float width,float height)
	{
		collisionShape = new RectangleShape(position.x, position.y, width,height);
	}

	ShapeType RigidBody2D:: GetShapeType() const {
		return collisionShape->type;
	}



	void RigidBody2D::move(const Vec2& displacement)
	{
		position += displacement;
		if (collisionShape)
			collisionShape->setPosition(position);
	}

	void RigidBody2D::moveTo(const Vec2& pos)
	{
		position = pos;
		if (collisionShape)
			collisionShape->setPosition(position);
	}


}