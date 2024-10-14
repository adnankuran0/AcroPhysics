#pragma once
#include "RigidBody.h"
#include <vector>
#include "Solver.h"

namespace acro {
	class PhysicsWorld
	{	
	public:
		Vec2 gravity = Vec2(0,9.8f);
		float timeScale = 10.0f;
		int maxIterations = 64;
		int minIterations = 1;
		void addBody(RigidBody* body);
		void removeBody(RigidBody* body);
		void step(float deltaTime,int iterations);
		int getBodyCount();
	private:
		std::vector<RigidBody*> bodies;

	};

}