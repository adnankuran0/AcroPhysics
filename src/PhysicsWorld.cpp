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

	int PhysicsWorld::getBodyCount() { return bodies.size(); }

	void PhysicsWorld::step(float deltaTime,int iterations)
	{
		int totalIterations = Math::clamp(iterations, minIterations, maxIterations);


		for (int currentIteration = 0; currentIteration < totalIterations; ++currentIteration)
		{
			Solver::handleCollision(bodies);

			for (RigidBody* body : bodies)
			{
				if (!body) continue;
				if (body->isStatic) continue;

				body->step(deltaTime,totalIterations, gravity, timeScale);
			}
		}

	}

}