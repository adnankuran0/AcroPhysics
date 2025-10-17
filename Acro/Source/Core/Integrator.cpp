#include "Core/Integrator.h"
#include <iostream>

using namespace Acro::Math;
using namespace Acro::Core;

void Integrator::Step(BodyManager& bodyManager, float deltaTime)
{
	size_t count = bodyManager.GetData().positions.size();
	for (size_t i = 0; i < count; i++)
	{
		IntegrateBody(bodyManager, i, deltaTime);
	}
}

void Integrator::IntegrateBody(BodyManager& bodyManager, size_t index, float deltaTime)
{
	auto& bodyData = bodyManager.GetData();
	float invMass = bodyData.inverseMasses[index];
	if (invMass <= 0.0f) return;

	Vector3& pos = bodyData.positions[index];
	Vector3& vel = bodyData.linearVelocities[index];
	Vector3& totalForce = bodyData.forceAccumulators[index];

	Vector3& angularVel = bodyData.angularVelocities[index];
	Quaternion& orientation = bodyData.orientations[index];

	Vector3 gravityForce = m_Gravity * 1.0f / (bodyData.inverseMasses[index]);
	totalForce += gravityForce;
	Vector3 acc = totalForce * invMass;

	vel += acc * deltaTime;
	pos += vel * deltaTime;

	totalForce = Vector3(0.0f);

	Vector3 deltaAngle = angularVel * deltaTime;
	float angle = deltaAngle.Length();
	Vector3 axis = (angle > 0.0f) ? deltaAngle / angle : Vector3(1.0f, 0.0f, 0.0f); 
	Quaternion deltaRot(axis, angle);
	orientation *= deltaRot;
	orientation.Normalize();

	bodyData.dirtyFlags[index] = 1;

}
