#include "Solver.h"

namespace acro
{
    static struct PairHash {
        template <typename T1, typename T2>
        std::size_t operator()(const std::pair<T1, T2>& pair) const {
            return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
        }
    };

    static struct PairEqual {
        template <typename T1, typename T2>
        bool operator()(const std::pair<T1, T2>& lhs, const std::pair<T1, T2>& rhs) const {
            return lhs.first == rhs.first && lhs.second == rhs.second;
        }
    };

    void Solver::handleCollision(std::vector<RigidBody*>& bodies)
    {
        std::unordered_set<std::pair<RigidBody*, RigidBody*>, PairHash, PairEqual> uniquePairs;
        for (size_t i = 0; i < bodies.size(); ++i) {
            for (size_t j = i + 1; j < bodies.size(); ++j) {
                if (bodies[i] && bodies[j]) {
                    uniquePairs.insert({ bodies[i], bodies[j] });
                }
            }
        }

        for (const auto& pair : uniquePairs) {
            if (!pair.first || !pair.second) continue;  

			if (pair.first->isStatic && pair.second->isStatic) continue;
			if (!intersectAABB(pair.first->collider->aabb , pair.second->collider->aabb)) continue; 

            if (pair.first->GetShapeType() == acro::ShapeType::CIRCLE && pair.second->GetShapeType() == acro::ShapeType::CIRCLE)
                resolveForCircleAndCircle(pair.first, pair.second);
            else if (pair.first->GetShapeType() == acro::ShapeType::RECTANGLE && pair.second->GetShapeType() == acro::ShapeType::CIRCLE)
                resolveForCircleAndRect(pair.second, pair.first);
            else if (pair.first->GetShapeType() == acro::ShapeType::CIRCLE && pair.second->GetShapeType() == acro::ShapeType::RECTANGLE)
                resolveForCircleAndRect(pair.first, pair.second);
            else
                resolveForRectAndRect(pair.first, pair.second);
        }
    }




    void Solver::resolveForCircleAndCircle(RigidBody* firstBody, RigidBody* secondBody)
    {
        if (!firstBody || !secondBody || !firstBody->collider || !secondBody->collider) return;  // nullptr kontrolü

        Vec2 normal = Vec2::zero;
        float depth = 0.0f;

		float distance = firstBody->position.distance(secondBody->position);
		float radiusSum = firstBody->collider->getRadius() + secondBody->collider->getRadius();

		if (distance > radiusSum) return;

		normal = (secondBody->position - firstBody->position).normalized();
		depth = radiusSum - distance;

		Solver::separateBodies(firstBody, secondBody, normal, depth);
		Solver::resolveCollision(firstBody, secondBody, normal, depth);
        
    }

    void Solver::resolveForCircleAndRect(RigidBody* circleBody, RigidBody* rectBody)
    {
        if (!circleBody || !rectBody || !circleBody->collider || !rectBody->collider) return;

		std::vector<Vec2>& vertices = rectBody->collider->transformedVertices;

		Vec2 circleCenter = circleBody->position;
		float radius = circleBody->collider->getRadius();

        Vec2 normal(0,0);
		float depth = FLT_MAX;

		Vec2 axis(0,0);
        float axisDepth = 0.0f;
		float minA, minB, maxA, maxB;

        for (int i = 0; i < vertices.size(); i++)
        {
            Vec2 p1 = vertices[i];
            Vec2 p2 = vertices[(i + 1) % vertices.size()];

            Vec2 edge = p2 - p1;
            axis = Vec2(-edge.y, edge.x).normalized();


            projectPolygonToAxis(vertices, axis, minA, maxA);
            projectCircleToAxis(circleCenter, radius, axis, minB, maxB);


            if (minA >= maxB || minB >= maxA) return;

            axisDepth = std::min(maxB - minA, maxA - minB);

            if (axisDepth < depth)
            {
                depth = axisDepth;
                normal = axis;
            }
        }

		//finding closest point
		int cpIndex = findClosetPointOnPolygon(vertices, circleCenter);
		Vec2 cp = vertices[cpIndex];

		axis = cp - circleCenter;
		axis = axis.normalized();

		projectPolygonToAxis(vertices, axis, minA, maxA);
		projectCircleToAxis(circleCenter, radius, axis, minB, maxB);

        if (minA >= maxB || minB >= maxA) return;

		axisDepth = std::min(maxB-minA, maxA-minB);

		if (axisDepth < depth)
		{
			depth = axisDepth;
			normal = axis;
		}

		Vec2 direction = rectBody->position - circleCenter;

		if (direction.dot(normal) < 0.0f)
        {
			normal = -normal;
		}

		Solver::separateBodies(circleBody, rectBody, normal, depth);
		Solver::resolveCollision(circleBody, rectBody, normal, depth);

    }

    void Solver::resolveForRectAndRect(RigidBody* firstBody, RigidBody* secondBody)
    {
        if (!firstBody || !secondBody || !firstBody->collider || !secondBody->collider) return;

        float minDepth = FLT_MAX;
        Vec2 normal(0,0);

        std::vector<Vec2>& verticesOfFirst = firstBody->collider->transformedVertices;
        std::vector<Vec2>& verticesOfSecond = secondBody->collider->transformedVertices;

        for (int i = 0; i < verticesOfFirst.size(); i++) {
            Vec2 p1 = verticesOfFirst[i];
            Vec2 p2 = verticesOfFirst[(i + 1) % verticesOfFirst.size()];

            Vec2 edge = p2 - p1;
            Vec2 axis = Vec2(-edge.y, edge.x).normalized();


            float minA = FLT_MAX, maxA = -FLT_MAX;
			Solver::projectPolygonToAxis(verticesOfFirst, axis, minA, maxA);

            
            float minB = FLT_MAX, maxB = -FLT_MAX;
			Solver::projectPolygonToAxis(verticesOfSecond, axis, minB, maxB);

            if (maxA < minB || maxB < minA) return;

            float overlap = std::min(maxA, maxB) - std::max(minA, minB);
            if (overlap < minDepth) {
                minDepth = overlap;
                normal = (maxA < maxB) ? axis : -axis;
            }
        }

        for (int i = 0; i < verticesOfSecond.size(); i++) {
            Vec2 p1 = verticesOfSecond[i];
            Vec2 p2 = verticesOfSecond[(i + 1) % verticesOfSecond.size()];

            Vec2 edge = p2 - p1;
            Vec2 axis = Vec2(-edge.y, edge.x).normalized();

            float minA = FLT_MAX, maxA = -FLT_MAX;
            Solver::projectPolygonToAxis(verticesOfFirst, axis, minA, maxA);

            float minB = FLT_MAX, maxB = -FLT_MAX;
			Solver::projectPolygonToAxis(verticesOfSecond, axis, minB, maxB);

            if (maxA < minB || maxB < minA) return;

            float overlap = std::min(maxA, maxB) - std::max(minA, minB);
            if (overlap < minDepth) {
                minDepth = overlap;
                normal = (maxA < maxB) ? axis : -axis;
            }
        }


		Solver::separateBodies(firstBody, secondBody, normal, minDepth);
		Solver::resolveCollision(firstBody, secondBody, normal, minDepth);
    }


    int Solver::findClosetPointOnPolygon(const std::vector<Vec2>& vertices, const Vec2& circleCenter)
    {
		//returns an index of the closest point on the polygon to the circle center
        int result = -1;
		float minDistance = FLT_MAX;    

        for (int i = 0; i < vertices.size(); i++)
        {
			Vec2 v = vertices[i];
			float distance = v.distance(circleCenter);  

			if (distance < minDistance)
			{
				minDistance = distance;
				result = i;
			}

        }

		return result;
    }

	void Solver::projectPolygonToAxis(const std::vector<Vec2>& vertices, const Vec2 axis, float& min, float& max)
	{
		min = FLT_MAX;
		max = FLT_MIN;

        for (int i = 0; i < vertices.size(); i++)
        {
			Vec2 v = vertices[i];
			float projection = v.dot(axis);

			if (projection < min) min = projection;
			if (projection > max) max = projection;
        }

    }

    void Solver::projectCircleToAxis(const Vec2& center, float radius, const Vec2 axis, float& min, float& max)
    {
		Vec2 direction = axis.normalized(); 
		Vec2 directionAndRadius = direction * radius;

		Vec2 p1 = center + directionAndRadius;
		Vec2 p2 = center - directionAndRadius;

		min = p1.dot(axis);
		max = p2.dot(axis);

		if (min > max) std::swap(min, max);
    }

	void Solver::resolveCollision(RigidBody* firstBody, RigidBody* secondBody, const Vec2& normal, float& depth)
	{
		if (!firstBody || !secondBody) return;

		Vec2 relativeVelocity = secondBody->velocity - firstBody->velocity;

        //if objects are already moving apart
		if (relativeVelocity.dot(normal) > 0) return;

		float e = std::min(firstBody->restitution, secondBody->restitution);
        float j = -(1+e) * relativeVelocity.dot(normal);

		j /= firstBody->inverseMass + secondBody->inverseMass;

		Vec2 impulse = normal * j;

		firstBody->velocity -= impulse * firstBody->inverseMass;
		secondBody->velocity += impulse * secondBody->inverseMass;
	}

	void Solver::separateBodies(RigidBody* firstBody, RigidBody* secondBody, const Vec2& normal, float& depth)
    {
		if (!firstBody || !secondBody) return;

        if (firstBody->isStatic)
        {
			secondBody->move(normal * depth);
        }
		else if (secondBody->isStatic)
		{
			firstBody->move(-normal * depth);
        }
        else
        {
			firstBody->move(-normal * (depth / 2.0f));
			secondBody->move(normal * (depth / 2.0f));

        }

	}   

	bool Solver::intersectAABB(const AABB& aabb1, const AABB& aabb2)
	{
		return (aabb1.m_Left <= aabb2.m_Right && aabb1.m_Right >= aabb2.m_Left &&
			aabb1.m_Top <= aabb2.m_Bottom && aabb1.m_Bottom >= aabb2.m_Top);
	}
}

