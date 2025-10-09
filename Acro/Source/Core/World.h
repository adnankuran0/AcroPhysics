#ifndef ACRO_WORLD_H
#define ACRO_WORLD_H

#include "Math/Vector3.h"
#include "Core/Rigidbody.h"
#include "Core/WorldData.h"
#include <vector>

using namespace Acro::Math;

namespace Acro {

class World
{
public:
	World(Vector3 gravity = { 0.0f,-9.8f,0.0f }) : m_Gravity(gravity) {}

	void Step(float deltaTime);

	inline void SetGravity(const Vector3& gravity) noexcept { m_Gravity = gravity; }
	inline Vector3 GetGravity() const noexcept { return m_Gravity; }

	int AddBody() 
	{	
		unsigned int index = m_WorldData.bodyCount;
		// TODO: use emplace_back
		m_WorldData.positions.emplace_back(Vector3(0.0f,0.0f,0.0f));
		m_WorldData.velocities.emplace_back(Vector3(0.0f,0.0f,0.0f));
		m_WorldData.accels.emplace_back(Vector3(0.0f,0.0f,0.0f));
		m_WorldData.bodyCount++;
		return index;
	}

	int AddBody(const Vector3& initialPos)
	{
		unsigned int index = m_WorldData.bodyCount;
		// TODO: use emplace_back
		m_WorldData.positions.emplace_back(initialPos);
		m_WorldData.velocities.emplace_back(Vector3(0.0f, 0.0f, 0.0f));
		m_WorldData.accels.emplace_back(Vector3(0.0f, 0.0f, 0.0f));
		m_WorldData.bodyCount++;
		return index;
	}

	Rigidbody& GetBody(int index)
	{
		Rigidbody body;

		body.SetPosition(m_WorldData.positions.at(index));
		body.SetVelocity(m_WorldData.velocities.at(index));
		body.SetAcceleration(m_WorldData.accels.at(index));

		return body;
	}

	void AddForce(int index, const Vector3& force)
	{
		Vector3 acc = m_WorldData.accels.at(index);
		acc += force;
		m_WorldData.accels[index] = acc;
	}

private:
	Vector3 m_Gravity;
	WorldData m_WorldData;
};

}

#endif // !ACRO_WORLD_H
