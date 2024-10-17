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

	void RigidBody::applyForce(const Vec2& f) {	force += f;	}

	void RigidBody::applyForce(float x, float y) {	force += Vec2(x,y);	}

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
		}
		else
		{
			std::cout << "No collision shape set for the rigid body" << std::endl;
		}


		force = Vec2(0, 0);
	}

	float RigidBody::calculateMomentOfInertia()
	{
		if (GetShapeType() == ShapeType::CIRCLE)
		{
			CircleShape* circle = dynamic_cast<CircleShape*>(collider);
			return 0.5f * mass * circle->getRadius() * circle->getRadius();
		}
		else if (GetShapeType() == ShapeType::RECTANGLE)
		{
			RectangleShape* rect = dynamic_cast<RectangleShape*>(collider);
			float w = rect->getWidth();
			float h = rect->getHeight();
			return (1.0f / 12.0f) * mass * (w * w + h * h);
		}
		else
		{
			return 0;
		}
	}

	void RigidBody::setCollider(float radius)
	{
		collider = new CircleShape(position.x,position.y,radius);
		interia = calculateMomentOfInertia();
		inverseInteria = 1.0f / interia;
	}

	void RigidBody::setCollider(float width,float height)
	{
		collider = new RectangleShape(position.x, position.y, width,height);
		interia = calculateMomentOfInertia();
		inverseInteria = 1.0f / interia;

	}

	Collider* RigidBody::getCollider() { return collider; }

	void RigidBody::setPosition(const Vec2& pos)
	{
		position = pos;
		if (collider)
			collider->setPosition(position);
	}

	void RigidBody::setPosition(float x, float y)
	{
		position = Vec2(x, y);
		if (collider)
			collider->setPosition(position);
	}

	Vec2 RigidBody::getPosition() const { return position; }

	Vec2 RigidBody::getVelocity() const { return velocity; }

	void RigidBody::setVelocity(const Vec2& vel) { velocity = vel; }

	void RigidBody::setVelocity(float x, float y) {	velocity = Vec2(x, y); }

	void RigidBody::setForce(const Vec2& f) { force = f; }

	void RigidBody::setForce(float x, float y) { force = Vec2(x, y); }

	Vec2 RigidBody::getForce() const { return force; }

	void RigidBody::setRotation(float r)
	{
		rotation = r;
		if (collider)
			collider->setRotation(rotation);
	}

	float RigidBody::getRotation() const { return rotation;	}

	void RigidBody::setRotationalVelocity(float v) { rotationalVelocity = v; }

	float RigidBody::getRotationalVelocity() const { return rotationalVelocity;	}


	void RigidBody::setMass(float m)
	{
		mass = m;
		if (isStatic)
			inverseMass = 0;
		else
			inverseMass = 1.0f / mass;
	}

	float RigidBody::getMass() const { return mass;	}	

	void RigidBody::setInverseMass(float invM) { inverseMass = invM; }

	float RigidBody::getInverseMass() const { return inverseMass; }

	void RigidBody::setInteria(float i) { interia = i; }

	float RigidBody::getInteria() const { return interia; }

	float RigidBody::getInverseInteria() const { return inverseInteria; }

	void RigidBody::setStaticMode(bool isStatic)
	{
	this->isStatic = isStatic;
		if (isStatic)
			inverseMass = 0;
		else
			inverseMass = 1.0f / mass;
	}

	void RigidBody::applyGravity(bool useGravity) {	this->useGravity = useGravity; }

	void RigidBody::setRestitution(float r) { restitution = r; }

	float RigidBody::getRestitution() const { return restitution; }

	ShapeType RigidBody:: GetShapeType() const { return collider->type;	}

	bool RigidBody::getIsStatic() const { return isStatic; }

}