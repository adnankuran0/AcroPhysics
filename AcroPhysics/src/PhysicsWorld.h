#pragma once
#include "RigidBody.h"
#include <vector>
#include "Solver.h"

namespace acro {
	class PhysicsWorld
	{	
	public:
		void addBody(RigidBody* body);
		void removeBody(RigidBody* body);
		void step(float deltaTime,int iterations);
		int getBodyCount();

		void setMinIterations(int minIterations);
		void setMaxIterations(int maxIterations);
		void setTimeScale(float timeScale);

		std::vector<Vec2> getContactPoints() const;
		void clearContactPoints();

		void setGravity(Vec2 gravity);
		Vec2 getGravity() const;

		int getMinIterations() const;
		int getMaxIterations() const;
		float getTimeScale() const;
	private:
		std::vector<RigidBody*> bodies;
		int minIterations = 1;
		int maxIterations = 64;
		float timeScale = 10.0f;
		Vec2 gravity = Vec2(0, 9.8f);



	};

}