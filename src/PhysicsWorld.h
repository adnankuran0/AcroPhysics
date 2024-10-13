#pragma once
#include "RigidBody.h"
#include <vector>
#include "Solver.h"

namespace acro {
	class PhysicsWorld
	{	
	public:
		Vec2 gravity = Vec2(0,9.8f);
		float timeScale = 100.0f;
		void addBody(RigidBody* body);
		void removeBody(RigidBody* body);
		void step(float deltaTime);
	private:
		std::vector<RigidBody*> bodies;

	};

}