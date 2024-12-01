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
		static std::vector<Vec2> contactPoints;

	private:
		static void resolveForCircleAndCircle(RigidBody* firstBody, RigidBody* secondBody);
		static void resolveForCircleAndRect(RigidBody* circleBody, RigidBody* rectBody);
		static void resolveForRectAndRect(RigidBody* firstBody, RigidBody* secondBody);

		static int findClosetPointOnPolygon(const std::vector<Vec2>& vertices, const Vec2& circleCenter);
		static void projectPolygonToAxis(const std::vector<Vec2>& vertices, const Vec2 axis, float& min, float& max);
		static void projectCircleToAxis(const Vec2& center, float radius, const Vec2 axis, float& min, float& max);

		static void separateBodies(RigidBody* firstBody, RigidBody* secondBody, const Vec2& normal, float& depth);
		static void resolveCollision(RigidBody* firstBody, RigidBody* secondBody, const Vec2& normal, float& depth);
		static void resolveCollisionWithRotation(RigidBody* firstBody, RigidBody* secondBody, const Vec2& normal, Vec2* contactPoint1, Vec2* contactPoint2);

		static bool intersectAABB(const AABB& aabb1, const AABB& aabb2);

		static void pointSegmentDistance(const Vec2& point, const Vec2& start, const Vec2& end, float& distanceSq, Vec2& closestPoint);

		static Vec2 findContactPointForCircles(const Vec2& center1, const Vec2& center2, float radius1, float radius2);
		static Vec2 findContactPointForCircleAndRect(const Vec2& circleCenter,float circleRadius,
			const Vec2& rectCenter, const std::vector<Vec2>& rectVertices);
		static std::vector<Vec2> findContactPointForRects(const std::vector<Vec2>& vertices1, const std::vector<Vec2>& vertices2);



	};
}
