#pragma once
#include "RigidBody.h"
#include <vector>
#include <set>
#include <utility>
#include <unordered_set>
#include "Collider.h"
#include <unordered_set>
#include <utility>

namespace acro
{
	class Solver
	{
	public:
		static void handleCollision(std::vector<RigidBody*>& bodies);

	private:
		static void resolveForCircleAndCircle(RigidBody* firstBody, RigidBody* secondBody);
		static void resolveForCircleAndRect(RigidBody* circleBody, RigidBody* rectBody);
		static void resolveForRectAndRect(RigidBody* firstBody, RigidBody* secondBody);

		static int findClosetPointOnPolygon(const std::vector<Vec2>& vertices, const Vec2& circleCenter);
		static void projectPolygonToAxis(const std::vector<Vec2>& vertices, const Vec2 axis, float& min, float& max);
		static void projectCircleToAxis(const Vec2& center, float radius, const Vec2 axis, float& min, float& max);

		static void separateBodies(RigidBody* firstBody, RigidBody* secondBody, const Vec2& normal, float& depth);
		static void resolveCollision(RigidBody* firstBody, RigidBody* secondBody, const Vec2& normal, float& depth);
	};
}
