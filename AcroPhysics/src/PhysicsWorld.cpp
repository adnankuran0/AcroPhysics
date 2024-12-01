#include "PhysicsWorld.h"

namespace acro {
	void PhysicsWorld::addBody(RigidBody* body)
	{

		if (!body->getCollider())
		{
			std::cerr << "RigidBody has no collider" << std::endl;
			return;
		}
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
				if (body->getIsStatic()) continue;

				body->step(deltaTime,totalIterations, gravity, timeScale);
			}
		}

	}

	std::vector<Vec2> PhysicsWorld::getContactPoints() const
	{
		return Solver::contactPoints;
	}

	void PhysicsWorld::clearContactPoints()
	{
		Solver::contactPoints.clear();
	}

	void PhysicsWorld::setMinIterations(int minIterations)
	{
		this->minIterations = minIterations;
	}

	void PhysicsWorld::setMaxIterations(int maxIterations)
	{
		this->maxIterations = maxIterations;
	}

	void PhysicsWorld::setTimeScale(float timeScale)
	{
		this->timeScale = timeScale;
	}

	int PhysicsWorld::getMinIterations() const
	{
		return minIterations;
	}

	int PhysicsWorld::getMaxIterations() const
	{
		return maxIterations;
	}

	float PhysicsWorld::getTimeScale() const
	{
		return timeScale;
	}

	void PhysicsWorld::setGravity(Vec2 gravity)
	{
		this->gravity = gravity;
	}

	Vec2 PhysicsWorld::getGravity() const
	{
		return gravity;
	}


}