#include "Solver.h"

namespace acro
{
    void Solver::handleCollision(std::vector<RigidBody2D*>& bodies)
    {
        std::set<std::pair<RigidBody2D*, RigidBody2D*>> uniquePairs;
        for (size_t i = 0; i < bodies.size(); ++i) {
            for (size_t j = i + 1; j < bodies.size(); ++j) {
                if (bodies[i] && bodies[j]) {  
                    uniquePairs.insert({ bodies[i], bodies[j] });
                }
            }
        }

        for (const auto& pair : uniquePairs) {
            if (!pair.first || !pair.second) continue;  

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




    void Solver::resolveForCircleAndCircle(RigidBody2D* firstBody, RigidBody2D* secondBody)
    {
        if (!firstBody || !secondBody || !firstBody->collisionShape || !secondBody->collisionShape) return;  // nullptr kontrolü

        Vec2 pos1 = firstBody->position;
        Vec2 pos2 = secondBody->position;

        Vec2 collisionVector = pos1 - pos2;
        float distance = pos1.distance(pos2);
        float minDistance = firstBody->collisionShape->getRadius() + secondBody->collisionShape->getRadius();

        if (distance < minDistance) {
            float overlap = minDistance - distance;
            Vec2 collisionNormal = collisionVector.normalized();
            Vec2 correctionVector = collisionNormal * (overlap / 2);

            float mass1 = firstBody->mass;
            float mass2 = secondBody->mass;

            Vec2 velocity1 = firstBody->velocity;
            Vec2 velocity2 = secondBody->velocity;

            float v1Normal = velocity1.dot(collisionNormal);
            float v2Normal = velocity2.dot(collisionNormal);

            float momentum1 = (v1Normal * (mass1 - mass2) + 2 * mass2 * v2Normal) / (mass1 + mass2);
            float momentum2 = (v2Normal * (mass2 - mass1) + 2 * mass1 * v1Normal) / (mass1 + mass2);

            Vec2 newVelocity1 = velocity1 + collisionNormal * (momentum1 - v1Normal);
            Vec2 newVelocity2 = velocity2 + collisionNormal * (momentum2 - v2Normal);

            //firstBody->velocity = newVelocity1;
            //secondBody->velocity = newVelocity2;

            if (!firstBody->isStatic) firstBody->position += correctionVector;
            if (!secondBody->isStatic) secondBody->position -= correctionVector;
        }
    }

    void Solver::resolveForCircleAndRect(RigidBody2D* circleBody, RigidBody2D* rectBody)
    {
        if (!circleBody || !rectBody || !circleBody->collisionShape || !rectBody->collisionShape) return;

		std::vector<Vec2>& vertices = rectBody->collisionShape->transformedVertices;

		Vec2 center = circleBody->position;
		float radius = circleBody->collisionShape->getRadius();

        Vec2 normal = Vec2::zero;
		float depth = FLT_MAX;

		Vec2 axis = Vec2::zero;
        float axisDepth = 0.0f;
		float minA, minB, maxA, maxB;

        for (int i = 0; i < vertices.size(); i++)
        {
			Vec2 p1 = vertices[i];
			Vec2 p2 = vertices[(i + 1) % vertices.size()];

			Vec2 edge = p2 - p1;
			axis = Vec2(-edge.y, edge.x).normalized();


			projectPolygonToAxis(vertices, axis, minA, maxA);
			projectCircleToAxis(center, radius, axis, minB, maxB);  


			if (minA >= maxB || minB >= maxA) return;

            //check later
			axisDepth = std::min(maxA, maxB) - std::max(minA, minB);

			if (axisDepth < depth)
			{
				depth = axisDepth;
                normal = axis;
            }

			//finding closest point
			int cpIndex = findClosetPointOnPolygon(vertices, center);
			Vec2 cp = vertices[cpIndex];

			axis = cp - center;
			axis = axis.normalized();

			projectPolygonToAxis(vertices, axis, minA, maxA);
			projectCircleToAxis(center, radius, axis, minB, maxB);

            if (minA >= maxB || minB >= maxA) return;

			axisDepth = std::min(maxA, maxB) - std::max(minA, minB);

			if (axisDepth < depth)
			{
				depth = axisDepth;
				normal = axis;
			}

			Vec2 direction = rectBody->position - center;

			if (direction.dot(normal) < 0)
            {
				normal = -normal;
			}

            Vec2 resolution = -normal * (depth / 2);
            circleBody->move(resolution);
            rectBody->move(-resolution);

        }

    }

    void Solver::resolveForRectAndRect(RigidBody2D* firstBody, RigidBody2D* secondBody)
    {
        if (!firstBody || !secondBody || !firstBody->collisionShape || !secondBody->collisionShape) return;

        float minDepth = FLT_MAX;
        Vec2 bestNormal(0,0);

        std::vector<Vec2>& verticesOfFirst = firstBody->collisionShape->transformedVertices;
        std::vector<Vec2>& verticesOfSecond = secondBody->collisionShape->transformedVertices;

        for (int i = 0; i < verticesOfFirst.size(); i++) {
            Vec2 p1 = verticesOfFirst[i];
            Vec2 p2 = verticesOfFirst[(i + 1) % verticesOfFirst.size()];

            Vec2 edge = p2 - p1;
            Vec2 normal = Vec2(-edge.y, edge.x).normalized();

            float minA = FLT_MAX, maxA = -FLT_MAX;
            for (Vec2 vertex : verticesOfFirst) {
                float projection = vertex.dot(normal);
                minA = std::min(minA, projection);
                maxA = std::max(maxA, projection);
            }

            float minB = FLT_MAX, maxB = -FLT_MAX;
            for (Vec2 vertex : verticesOfSecond) {
                float projection = vertex.dot(normal);
                minB = std::min(minB, projection);
                maxB = std::max(maxB, projection);
            }

            if (maxA < minB || maxB < minA) return;

            float overlap = std::min(maxA, maxB) - std::max(minA, minB);
            if (overlap < minDepth) {
                minDepth = overlap;
                bestNormal = (maxA < maxB) ? normal : -normal;
            }
        }

        for (int i = 0; i < verticesOfSecond.size(); i++) {
            Vec2 p1 = verticesOfSecond[i];
            Vec2 p2 = verticesOfSecond[(i + 1) % verticesOfSecond.size()];

            Vec2 edge = p2 - p1;
            Vec2 normal = Vec2(-edge.y, edge.x).normalized();

            float minA = FLT_MAX, maxA = -FLT_MAX;
            for (Vec2 vertex : verticesOfFirst) {
                float projection = vertex.dot(normal);
                minA = std::min(minA, projection);
                maxA = std::max(maxA, projection);
            }

            float minB = FLT_MAX, maxB = -FLT_MAX;
            for (Vec2 vertex : verticesOfSecond) {
                float projection = vertex.dot(normal);
                minB = std::min(minB, projection);
                maxB = std::max(maxB, projection);
            }

            if (maxA < minB || maxB < minA) return;

            float overlap = std::min(maxA, maxB) - std::max(minA, minB);
            if (overlap < minDepth) {
                minDepth = overlap;
                bestNormal = (maxA < maxB) ? normal : -normal;
            }
        }

        Vec2 resolution = -bestNormal * (minDepth / 2);
        firstBody->move(resolution);
        secondBody->move(-resolution);
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

	void Solver::projectPolygonToAxis(const std::vector<Vec2>& vertices, const Vec2& axis, float& min, float& max)
	{
		min = FLT_MAX;
		max = -FLT_MAX;

        for (int i = 0; i < vertices.size(); i++)
        {
			Vec2 v = vertices[i];
			float projection = v.dot(axis);

			if (projection < min) min = projection;
			if (projection > max) max = projection;
        }

    }

    void Solver::projectCircleToAxis(const Vec2& center, float radius, const Vec2& axis, float& min, float& max)
    {
		Vec2 direction = axis.normalized(); 
		Vec2 directionAndRadius = direction * radius;

		Vec2 pol1 = center + directionAndRadius;
		Vec2 pol2 = center - directionAndRadius;

		min = pol1.dot(axis);
		max = pol2.dot(axis);

		if (min > max) std::swap(min, max);
    }
}

