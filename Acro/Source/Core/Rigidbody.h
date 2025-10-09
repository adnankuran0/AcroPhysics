#ifndef ACRO_RIGIDBODY_H
#define ACRO_RIGIDBODY_H

#include "Math/Vector3.h"

using namespace Acro::Math;

namespace Acro {

class Rigidbody
{
public:

	inline void SetPosition(const Vector3& pos) noexcept { m_Position = pos; }
	inline Vector3 GetPosition() const noexcept { return m_Position; }

	inline void SetVelocity(const Vector3& vel) noexcept { m_Velocity = vel; }
	inline Vector3 GetVelocity() const noexcept { return m_Velocity; }

	inline void SetAcceleration(const Vector3& acc) noexcept { m_Acceleration = acc; }
	inline Vector3 GetAcceleration() const noexcept { return m_Acceleration; }


private:
	Vector3 m_Position;
	Vector3 m_Velocity;
	Vector3 m_Acceleration;
};

}

#endif // !ACRO_RIGIDBODY_H
