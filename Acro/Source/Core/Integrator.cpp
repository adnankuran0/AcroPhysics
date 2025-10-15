#include "Core/Integrator.h"
#include <iostream>

using namespace Acro::Math;
using namespace Acro::Core;

void Integrator::Step(BodyData& bodyData, float deltaTime)
{
	size_t count = bodyData.positions.size();
	for (size_t i = 0; i < count; i++)
	{
		IntegrateBody(bodyData, i, deltaTime);
	}
}

void Integrator::IntegrateBody(BodyData& bodyData, size_t index, float deltaTime)
{
	Vector3& pos = bodyData.positions[index];
	Vector3& vel = bodyData.linearVelocities[index];
	Vector3& totalForce = bodyData.forceAccumulators[index];

	Vector3& angularVel = bodyData.angularVelocities[index];
	Quaternion& orientation = bodyData.orientations[index];



	Vector3 acc;
	float invMass = bodyData.inverseMasses[index];

	if (invMass > 0.0f)
	{
		Vector3 gravityForce = m_Gravity * 1.0f / (bodyData.inverseMasses[index]);
		totalForce += gravityForce;
		acc = totalForce * invMass;
	}
	else
		return;
		//acc = Vector3(0.0f);

	vel += acc * deltaTime;
	pos += vel * deltaTime;

	totalForce = Vector3(0.0f);

	Vector3 deltaAngle = angularVel * deltaTime;
	float angle = deltaAngle.Length();
	Vector3 axis = (angle > 0.0f) ? deltaAngle / angle : Vector3(1.0f, 0.0f, 0.0f); 
	Quaternion deltaRot(axis, angle);
	orientation *= deltaRot;
	orientation.Normalize();

}
