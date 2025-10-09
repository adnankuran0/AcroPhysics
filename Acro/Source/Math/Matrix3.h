#ifndef ACRO_MATH_MAT3_H
#define ACRO_MATH_MAT3_H

#include "glm/glm.hpp"
#include "Math/Vector3.h"

namespace Acro::Math {
	
class Matrix3
{
public:
	float m[3][3];

	Matrix3()
	{
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				m[i][j] = 0.0f;
			}
		}
	}

	Matrix3(float diagonal)
	{
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				m[i][j] = (i == j) ? diagonal : 0.0f;
			}
		}
	}

	Matrix3(const glm::mat3& mat)
	{
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				m[i][j] = mat[i][j];
			}
		}
	}

	operator glm::mat3() const
	{
		glm::mat3 result;
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				result[i][j] = m[i][j];
			}
		}
	}

	inline static Matrix3 Identity() { return Matrix3(1.0f); }

	Matrix3 operator*(const Matrix3& other) const
	{
		glm::mat3 a = *this;
		glm::mat3 b = other;
		glm::mat3 result = a * b;
		return Matrix3(result);
	}

	Vector3 operator*(const Vector3& v) const
	{
		glm::mat3 a = *this;
		glm::vec3 b = v;
		glm::vec3 result = a * b;
		return Vector3(result);
	}

	Matrix3 Transpose() const
	{
		return glm::transpose(glm::mat3(*this));
	}

	Matrix3 Inverse() const
	{
		return glm::inverse(glm::mat3(*this));
	}

};

}

#endif // !ACRO_MATH_MAT3_H

