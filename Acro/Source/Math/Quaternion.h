#ifndef ACRO_MATH_QUATERNION_H
#define ACRO_MATH_QUATERNION_H

#define GLM_ENABLE_EXPERIMENTAL
#include "glm/glm.hpp"
#include "glm/gtc/quaternion.hpp"
#include "glm/gtx/quaternion.hpp"
#include "Math/Vector3.h"

namespace Acro::Math {

class Quaternion
{
public:
	Quaternion() noexcept : q(1.0f,0.0f,0.0f,0.0f) {}
	Quaternion(float w, float x, float y, float z) noexcept : q(w,x,y,z) {}
	Quaternion(const glm::quat& other) noexcept : q(other) {}
	Quaternion(const Vector3& axis, float angleRadians) noexcept
	{
		glm::vec3 glmAxis(axis.x, axis.y, axis.z);
		q = glm::angleAxis(angleRadians, glm::normalize(glmAxis));
	}

	static Quaternion Identity() noexcept { return Quaternion(); }

	Quaternion operator*(const Quaternion& other) const noexcept
	{
		return Quaternion(q * other.q);
	}

	Quaternion& operator*=(const Quaternion& other) 
	{
		q *= other.q;
		return *this;
	}

	inline void Normalize() noexcept { q = glm::normalize(q); }
	inline Quaternion Normalized() const noexcept { return Quaternion(glm::normalize(q)); }
	inline Quaternion Inverse() const noexcept { return Quaternion(glm::inverse(q)); }

	inline Vector3 Rotate(const Vector3& v) const noexcept
	{
		return Vector3(glm::rotate(q, glm::vec3(v.x,v.y,v.z)));
	}

	inline glm::mat4 ToMat4() const noexcept { return glm::toMat4(q); }
	inline glm::mat3 ToMat3() const noexcept { return glm::toMat3(q); }
	inline Vector3 ToEuler() const noexcept { return Vector3(glm::eulerAngles(q)); }

	operator glm::quat() const noexcept { return q; }
	

private:
	glm::quat q;
};

}

#endif // ACRO_MATH_QUATERNION_H