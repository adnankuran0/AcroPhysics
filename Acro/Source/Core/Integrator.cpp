#include "Core/Integrator.h"

#include <iostream>

void Acro::Integrator::Step(BodyData& bodyData, Vector3& gravity ,float deltaTime)
{
	size_t count = bodyData.positions.size();
	for (size_t i = 0; i < count; i++)
	{
		IntegrateBody(bodyData, gravity,i, deltaTime);
	}
}

void Acro::Integrator::IntegrateBody(BodyData& bodyData, Vector3& gravity ,size_t index, float deltaTime)
{
	Vector3& pos = bodyData.positions[index];
	Vector3& vel = bodyData.linearVelocities[index];
	Vector3& totalForce = bodyData.forceAccumulators[index];

	Vector3 gravityForce = gravity * bodyData.masses[index];

	totalForce += gravityForce + totalForce;

	Vector3 acc = totalForce / bodyData.masses[index];

	vel += acc * deltaTime;
	pos += vel * deltaTime;

	totalForce = Vector3(0.0f);

}
