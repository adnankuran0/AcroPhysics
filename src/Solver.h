#pragma once
#include "RigidBody2D.h"
#include <vector>
#include <set>
#include <utility>
#include <unordered_set>
#include "CollisionShape.h"
#include <unordered_set>
#include <utility>

namespace acro
{
	class Solver
	{
	public:
		static void handleCollision(std::vector<RigidBody2D*>& bodies);
	private:
		static void resolveForCircleAndCircle(RigidBody2D* firstBody, RigidBody2D* secondBody);
		static void resolveForCircleAndRect(RigidBody2D* circleBody, RigidBody2D* rectBody);
		static void resolveForRectAndRect(RigidBody2D* firstBody, RigidBody2D* secondBody);
		static int findClosetPointOnPolygon(const std::vector<Vec2>& vertices, const Vec2& circleCenter);
		static void projectPolygonToAxis(const std::vector<Vec2>& vertices, const Vec2& axis, float& min, float& max);
		static void projectCircleToAxis(const Vec2& center, float radius, const Vec2& axis, float& min, float& max);
	};
}
