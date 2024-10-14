#include "RigidBody.h"

namespace acro {
	RigidBody::RigidBody(Vec2 pos, float m, bool isStatic,float restitution) : position(pos), velocity(Vec2(0,0)), 
		mass(m), force(0, 0), isStatic(isStatic),restitution(restitution), rotation(0), rotationalVelocity(0)
	{
		if (isStatic)
			inverseMass = 0;
		else
			inverseMass = 1.0f / mass;
	}

	RigidBody::RigidBody(const RigidBody& other) = default;
	
	RigidBody::~RigidBody()
	{
		//delete collider;
		
	}

	void RigidBody::applyForce(const Vec2& f)
	{
		force += f;
	}

	void RigidBody::applyForce(float x, float y)
	{
		force += Vec2(x,y);
	}

	void RigidBody::setCollider(float radius)
	{
		collider = new CircleShape(position.x,position.y,radius);
	}

	void RigidBody::setCollider(float width,float height)
	{
		collider = new RectangleShape(position.x, position.y, width,height);
	}

	ShapeType RigidBody:: GetShapeType() const {
		return collider->type;
	}


	void RigidBody::move(const Vec2& displacement)
	{
		position += displacement;
		if (collider)
			collider->setPosition(position);
	}

	void RigidBody::moveTo(const Vec2& pos)
	{
		position = pos;
		if (collider)
			collider->setPosition(position);
	}

	void RigidBody::step(float deltaTime,int iterations, const Vec2& gravity,float timeScale)
	{
		float dt = deltaTime / (float)iterations;

		force *= iterations;

		Vec2 netForce = (useGravity) ? (gravity * mass) + force: force;

		Vec2 acceleration = netForce  / mass;

		velocity += acceleration * dt * timeScale;
		position += velocity * dt * timeScale;
		rotation += rotationalVelocity * dt * timeScale;

		if (collider)
		{
			collider->setPosition(position);
			collider->setRotation(rotation);
			collider->updateAABB();
		}
		else
		{
			std::cout << "No collision shape set for the rigid body" << std::endl;
		}


		force = Vec2(0, 0);
	}

}