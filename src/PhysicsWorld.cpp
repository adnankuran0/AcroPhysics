#include "PhysicsWorld.h"

namespace acro {
	void PhysicsWorld::addBody(RigidBody* body)
	{
		bodies.push_back(body);
	}

	void PhysicsWorld::removeBody(RigidBody* body)
	{
		if (!body) return;
		auto it = std::find(bodies.begin(), bodies.end(), body);
		if (it != bodies.end())
			bodies.erase(it);

	}

	void PhysicsWorld::step(float deltaTime)
	{
		Solver::handleCollision(bodies);

		for (RigidBody *body : bodies)
		{
			if (!body) continue;
			if (body->isStatic) continue;

			body->step(deltaTime,gravity,timeScale);
		}
	}

}