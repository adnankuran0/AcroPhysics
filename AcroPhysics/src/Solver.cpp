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

    std::vector<Vec2> Solver::contactPoints;

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

            if (!pair.first->getCollider() || !pair.second->getCollider()) continue;
			if (pair.first->getIsStatic() && pair.second->getIsStatic()) continue;
			if (!intersectAABB(pair.first->getCollider()->aabb , pair.second->getCollider()->aabb)) continue;

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
        if (!firstBody || !secondBody || !firstBody->getCollider() || !secondBody->getCollider()) return;  // nullptr kontrolü

        Vec2 normal = Vec2::zero;
        float depth = 0.0f;

		float distance = firstBody->getPosition().distance(secondBody->getPosition());
		float radiusSum = firstBody->getCollider()->getRadius() + secondBody->getCollider()->getRadius();

		if (distance > radiusSum) return;

		normal = (secondBody->getPosition() - firstBody->getPosition()).normalized();
		depth = radiusSum - distance;

		Solver::separateBodies(firstBody, secondBody, normal, depth);
		Vec2 contactPoint = Solver::findContactPointForCircles(firstBody->getPosition(), secondBody->getPosition(), firstBody->getCollider()->getRadius(),
            secondBody->getCollider()->getRadius());

		Solver::resolveCollision(firstBody, secondBody, normal, depth);
        contactPoints.push_back(contactPoint);

        
    }

    void Solver::resolveForCircleAndRect(RigidBody* circleBody, RigidBody* rectBody)
    {
        if (!circleBody || !rectBody || !circleBody->getCollider() || !rectBody->getCollider()) return;

		std::vector<Vec2>& vertices = rectBody->getCollider()->transformedVertices;

		Vec2 circleCenter = circleBody->getPosition();
		float radius = circleBody->getCollider()->getRadius();

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

		Vec2 direction = rectBody->getPosition() - circleCenter;

		if (direction.dot(normal) < 0.0f)
        {
			normal = -normal;
		}

		Solver::separateBodies(circleBody, rectBody, normal, depth);
		Vec2 contactPoint = Solver::findContactPointForCircleAndRect(circleBody->getPosition(), circleBody->getCollider()->getRadius()
            , rectBody->getPosition(), rectBody->getCollider()->transformedVertices);

		Solver::resolveCollision(circleBody, rectBody, normal,depth);

		contactPoints.push_back(contactPoint);

    }

    void Solver::resolveForRectAndRect(RigidBody* firstBody, RigidBody* secondBody)
    {
        if (!firstBody || !secondBody || !firstBody->getCollider() || !secondBody->getCollider()) return;

        float minDepth = FLT_MAX;
        Vec2 normal(0,0);

        std::vector<Vec2>& verticesOfFirst = firstBody->getCollider()->transformedVertices;
        std::vector<Vec2>& verticesOfSecond = secondBody->getCollider()->transformedVertices;

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
		Vec2 contactPoint1 = Vec2::zero;
		Vec2 contactPoint2 = Vec2::zero;

		std::vector<Vec2> cps = Solver::findContactPointForRects(verticesOfFirst, verticesOfSecond);
		int cpCount = cps.size();

		if (cpCount == 1)
		{
            contactPoint1 = cps[0];
            contactPoints.push_back(contactPoint1);
		}
		else if (cpCount == 2)
		{
            contactPoint1 = cps[0];
            contactPoint2 = cps[1];
            contactPoints.push_back(contactPoint1);
            contactPoints.push_back(contactPoint2);
		}


		Solver::resolveCollision(firstBody, secondBody, normal,minDepth);

    }

	void Solver::resolveCollision(RigidBody* firstBody, RigidBody* secondBody, const Vec2& normal, float& depth)
	{
		if (!firstBody || !secondBody) return;

		Vec2 relativeVelocity = secondBody->getVelocity() - firstBody->getVelocity();

        //if objects are already moving apart
		if (relativeVelocity.dot(normal) > 0) return;

		float e = std::min(firstBody->getRestitution(), secondBody->getRestitution());
        float j = -(1+e) * relativeVelocity.dot(normal);

		j /= firstBody->getInverseMass() + secondBody->getInverseMass();

		Vec2 impulse = normal * j;

		firstBody->setVelocity(firstBody->getVelocity() - impulse * firstBody->getInverseMass());
		secondBody->setVelocity(secondBody->getVelocity() + impulse * secondBody->getInverseMass());
	}

    void Solver::resolveCollisionWithRotation(RigidBody* firstBody, RigidBody* secondBody, const Vec2& normal, Vec2* contactPoint1, Vec2* contactPoint2)
    {
		if (!firstBody || !secondBody) return;
		if (firstBody->getIsStatic() || secondBody->getIsStatic()) return;
        

        int contactCount = 0;
		if (!contactPoint2) contactCount = 1;
		else contactCount = 2;

        Vec2 contactList[2];
		Vec2 impulseList[2];
        Vec2 raList[2];
		Vec2 rbList[2];

		float e = fmin(firstBody->getRestitution(), secondBody->getRestitution());

		contactList[0] = *contactPoint1;
		if (contactPoint2)
		    contactList[1] = *contactPoint2;

        for (int i = 0; i < contactCount; i++)
        {
			impulseList[i] = Vec2::zero;
			raList[i] = contactList[i] - firstBody->getPosition();
			rbList[i] = contactList[i] - secondBody->getPosition();

        }

		for (int i = 0; i < contactCount; i++)
		{
			Vec2 ra = contactList[i] - firstBody->getPosition();
			Vec2 rb = contactList[i] - secondBody->getPosition();

			raList[i] = ra;
			rbList[i] = rb;

			Vec2 raPerp = Vec2(-ra.y, ra.x);
			Vec2 rbPerp = Vec2(-rb.y, rb.x);

			Vec2 angularLinearVelocityA = raPerp * firstBody->getRotationalVelocity();
			Vec2 angularLinearVelocityB = rbPerp * secondBody->getRotationalVelocity();

			Vec2 relativeVelocity = (secondBody->getVelocity() + angularLinearVelocityB)
                - (firstBody->getVelocity() - angularLinearVelocityA);

			float contactVelocityMag = relativeVelocity.dot(normal);
            if (contactVelocityMag > 0.0f)
                continue;

			float raPerpDotN = raPerp.dot(normal);
			float rbPerpDotN = rbPerp.dot(normal);

			float denom = firstBody->getInverseMass() + secondBody->getInverseMass() 
                + (raPerpDotN * raPerpDotN) * firstBody->getInverseInertia() 
                + (rbPerpDotN * rbPerpDotN) * secondBody->getInverseInertia();

			float j = -(1.0f + e) * contactVelocityMag;
			j /= denom;
			j /= static_cast<float>(contactCount);

			Vec2 impulse = normal * j;
			impulseList[i] = impulse;
        }

		for (int i = 0; i < contactCount; i++)
		{
			Vec2 impulse = impulseList[i];
			std::cout << firstBody->getInverseInertia() << std::endl;
			Vec2 ra = raList[i];
			Vec2 rb = rbList[i];

            firstBody->setVelocity(firstBody->getVelocity() + (-impulse) * firstBody->getInverseMass());
			firstBody->setRotationalVelocity(firstBody->getRotationalVelocity() + firstBody->getInverseInertia() * ra.cross(-impulse));
			secondBody->setVelocity(secondBody->getVelocity() + impulse * secondBody->getInverseMass());
			secondBody->setRotationalVelocity(secondBody->getRotationalVelocity() + secondBody->getInverseInertia() * rb.cross(impulse));
        }
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

	void Solver::separateBodies(RigidBody* firstBody, RigidBody* secondBody, const Vec2& normal, float& depth)
    {
		if (!firstBody || !secondBody) return;

        if (firstBody->getIsStatic())
        {
			secondBody->move(normal * depth);
        }
		else if (secondBody->getIsStatic())
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

    void Solver::pointSegmentDistance(const Vec2& point, const Vec2& start, const Vec2& end, float& distanceSq, Vec2& closestPoint)
    {
		Vec2 ab = end - start;
		Vec2 ap = point - start;

		float proj = ap.dot(ab);
		float abLengthSq = ab.magnitudeSquared();
		float d = proj / abLengthSq;

        if (d <= 0.0f)
        {
            closestPoint = start;
        }
		else if (d >= 1.0f)
		{
			closestPoint = end;
		}
		else
		{
			closestPoint = start + ab * d;
		}

		distanceSq = (point - closestPoint).magnitudeSquared();
    }

	Vec2 Solver::findContactPointForCircles(const Vec2& center1, const Vec2& center2, float radius1, float radius2)
    {
		Vec2 normal = (center2 - center1).normalized();
		return center1 + normal * radius1;
	}

    Vec2 Solver::findContactPointForCircleAndRect(const Vec2& circleCenter, float circleRadius, 
        const Vec2& rectCenter, const std::vector<Vec2>& rectVertices)
    {
		Vec2 cp = Vec2::zero;

        float minDistanceSq = FLT_MAX;

        for (int i = 0; i < rectVertices.size(); i++)
        {
			Vec2 va = rectVertices[i];
			Vec2 vb = rectVertices[(i + 1) % rectVertices.size()];

            float distanceSq;
            Vec2 contact = Vec2::zero;

			pointSegmentDistance(circleCenter, va, vb, distanceSq, contact);

			if (distanceSq < minDistanceSq)
			{
                minDistanceSq = distanceSq;
				cp = contact;
            }
        }

        return cp;
    }

    std::vector<Vec2> Solver::findContactPointForRects(const std::vector<Vec2>& vertices1, const std::vector<Vec2>& vertices2)
    {
        Vec2 cp1 = Vec2::zero;
        Vec2 cp2 = Vec2::zero;
        float cpCount = 0;
        float minDistanceSq = FLT_MAX;

        auto checkContact = [&](const Vec2& va, const Vec2& vb, const std::vector<Vec2>& vertices) {
            for (int j = 0; j < vertices.size(); j++)
            {
                Vec2 vc = vertices[j];
                Vec2 vd = vertices[(j + 1) % vertices.size()];
                Vec2 contact = Vec2::zero;
                float distanceSq;

                pointSegmentDistance(va, vc, vd, distanceSq, contact);

                if (Math::nearlyEquals(distanceSq, minDistanceSq))
                {
                    if (!Vec2::nearlyEquals(cp1, contact))
                    {
                        cp2 = contact;
                        cpCount = 2;
                    }
                }
                else if (distanceSq < minDistanceSq)
                {
                    minDistanceSq = distanceSq;
                    cpCount = 1;
                    cp1 = contact;
                }
            }
            };

        for (int i = 0; i < vertices1.size(); i++)
        {
            Vec2 va = vertices1[i];
            Vec2 vb = vertices1[(i + 1) % vertices1.size()];
            checkContact(va, vb, vertices2);
        }

        for (int i = 0; i < vertices2.size(); i++)
        {
            Vec2 va = vertices2[i];
            Vec2 vb = vertices2[(i + 1) % vertices2.size()];
            checkContact(va, vb, vertices1);
        }

        std::vector<Vec2> result;
        if (cpCount >= 1)
        {
            result.push_back(cp1);
        }
        if (cpCount == 2)
        {
            result.push_back(cp2);
        }


        return result;
    }

}

