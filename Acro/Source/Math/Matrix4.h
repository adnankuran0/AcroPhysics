#ifndef ACRO_MATH_MATRIX4_H
#define ACRO_MATH_MATRIX4_H

#define GLM_ENABLE_EXPERIMENTAL
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtx/matrix_decompose.hpp"
#include <glm/ext/matrix_float4x4.hpp>

#include "Math/Vector3.h"
#include "Math/Quaternion.h"

namespace Acro::Math {

class Matrix4
{
public: 
	Matrix4() noexcept : m(1.0f) {}
	Matrix4(float diag) noexcept : m(diag) {}
	Matrix4(const glm::mat4& mat) noexcept : m(mat) {}

	inline static Matrix4 Identity() noexcept { return Matrix4(1.0f); }

	inline static Matrix4 Translation(const Vector3& trans) noexcept
	{
		return Matrix4(glm::translate(glm::mat4(1.0f),glm::vec3(trans.x,trans.y,trans.z)));
	}

	inline static Matrix4 Rotation(const Quaternion& quat) noexcept
	{
		return Matrix4(glm::toMat4(glm::quat(quat)));
	}

	inline static Matrix4 Scale(const Vector3& scale) noexcept
	{
		return Matrix4(glm::scale(glm::mat4(1.0f),glm::vec3(scale.x,scale.y,scale.z)));
	}

	inline Matrix4 operator*(const Matrix4& other) const noexcept
	{
		return Matrix4(m * other.m);
	}

	inline Vector3 operator*(const Vector3& vec) const noexcept
	{
		return TransformPoint(vec);
	}

	inline Matrix4& operator*=(const Matrix4& other) noexcept
	{
		m *= other.m;
		return *this;
	}

	inline Vector3 TransformPoint(const Vector3& vec) const noexcept
	{
		glm::vec4 result = m * glm::vec4(vec.x, vec.y, vec.z, 1.0f);
		return Vector3(result.x,result.y,result.z);
	}

	inline Vector3 TransformVector(const Vector3& vec) const noexcept
	{
		glm::vec4 result = m * glm::vec4(vec.x, vec.y, vec.z, 0.0f);
		return Vector3(result.x, result.y, result.z);
	}

	inline Matrix4 Inversed() const noexcept
	{
		return Matrix4(glm::inverse(m));
	}

	inline Matrix4 Transposed() const noexcept
	{
		return Matrix4(glm::transpose(m));
	}

	void Decompose(Vector3& outTranslation, Quaternion& outRotation, Vector3& outScale) const noexcept;

	inline operator glm::mat4() const noexcept { return m; }
	inline const glm::mat4& GetNative() const noexcept { return m; }

private:
	glm::mat4 m;
};

}


#endif // ACRO_MATH_MATRIX4_H