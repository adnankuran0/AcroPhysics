#include "PhysicsWorld2D.h"

namespace acro {
	void PhysicsWorld2D::addBody(RigidBody2D* body)
	{
		bodies.push_back(body);
	}

	void PhysicsWorld2D::removeBody(RigidBody2D* body)
	{
		if (!body) return;
		auto it = std::find(bodies.begin(), bodies.end(), body);
		if (it != bodies.end())
			bodies.erase(it);

	}

	void PhysicsWorld2D::step(float deltaTime)
	{
		Solver::handleCollision(bodies);

		for (RigidBody2D *body : bodies)
		{
			if (!body) continue;
			if (body->isStatic) continue;

			Vec2 netForce = (!body->isStatic) ? (gravity * body->mass) + body->force : body->force;

			// Vec2 acceleration = netForce / body->mass;
			Vec2 acceleration = body->force / body->mass;

			body->velocity += acceleration * deltaTime;
			body->position += body->velocity * deltaTime * timeScale;
			body->rotation += body->rotationalVelocity * deltaTime * timeScale;

			if (body->collisionShape)
			{
				body->collisionShape->setPosition(body->position);
				body->collisionShape->setRotation(body->rotation);
			}
				

			body->force = Vec2(0, 0);


		}
	}

}