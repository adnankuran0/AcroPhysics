#include "Core/World.h"

using namespace Acro;

void World::Step(float deltaTime)
{
	for (int bodyIndex = 0; bodyIndex < m_WorldData.bodyCount; bodyIndex++ )
	{
		Vector3 pos = m_WorldData.positions.at(bodyIndex);
		Vector3 vel = m_WorldData.velocities.at(bodyIndex);
		Vector3 acc = m_WorldData.accels.at(bodyIndex);

		acc += m_Gravity * deltaTime;
		vel += acc * deltaTime;
		pos += acc * deltaTime;

		m_WorldData.positions[bodyIndex] = pos;
		m_WorldData.velocities[bodyIndex] = vel;
		m_WorldData.accels[bodyIndex] = acc;
	}
}