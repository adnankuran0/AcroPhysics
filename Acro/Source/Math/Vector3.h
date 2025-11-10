#ifndef ACRO_MATH_VECTOR3_H
#define ACRO_MATH_VECTOR3_H

#include "glm/glm.hpp"
#include <algorithm>

namespace Acro::Math {
	
class Vector3
{
public:
	float x, y, z;
	constexpr Vector3() : x(0.0f) , y(0.0f), z(0.0f) {}
	constexpr Vector3(float x, float y, float z) : x(x), y(y), z(z) {}
	constexpr Vector3(const glm::vec3& v) : x(v.x) , y(v.y), z(v.z) {}
	constexpr Vector3(float value) : x(value), y(value), z(value) {}


	inline static constexpr Vector3 ZERO() { return Vector3(0.0); }

	inline operator glm::vec3() const noexcept { return glm::vec3(x, y, z); }

	inline float& operator[](int index) noexcept
	{
		switch (index)
		{
		case 0:
			return x;
			break;
		case 1:
			return y;
			break;
		case 2:
			return z;
			break;
		default:
			return x; 
			break;

		}
	}
	inline const float& operator[](int index) const noexcept
	{
		switch (index)
		{
		case 0:
			return x;
			break;
		case 1:
			return y;
			break;
		case 2:
			return z;
			break;
		default:
			return x;
			break;

		}
	}

	inline Vector3 operator+(const Vector3& other) const noexcept { return Vector3(x + other.x, y + other.y, z + other.z); }
	inline Vector3& operator+=(const Vector3& other)
	{
		x += other.x;
		y += other.y;
		z += other.z;
		return *this;
	}
	inline Vector3 operator-() const noexcept { return Vector3(-x,-y,-z); }
	inline Vector3 operator-(const Vector3& other) const noexcept { return Vector3(x - other.x, y - other.y, z - other.z); }
	inline Vector3& operator-=(const Vector3& other) noexcept
	{
		x -= other.x;
		y -= other.y;
		z -= other.z;
		return *this;
	}

	inline Vector3& operator*=(float value) noexcept 
	{
		x *= value;
		y *= value;
		z *= value;
    return *this;
	}

	inline Vector3 operator*(float value) const noexcept { return Vector3(x * value, y * value, z * value); }
	inline Vector3 operator/(float value) const noexcept { return Vector3(x / value, y / value, z / value); }

	inline static Vector3 Clamp(const Vector3& vec, const Vector3& min, const Vector3& max) noexcept 
	{
		return
		{
			std::clamp(vec.x,min.x,max.x),
			std::clamp(vec.y,min.y,max.y),
			std::clamp(vec.z,min.z,max.z)
		};
	}

	inline float Length() const noexcept { return glm::length(glm::vec3(x, y, z)); }
	inline float Length2() const noexcept { return glm::dot(glm::vec3(x, y, z), glm::vec3(x, y, z)); }
	inline Vector3 Normalized() const noexcept { return glm::normalize(glm::vec3(x, y, z)); }
	inline Vector3 Normalize() noexcept 
	{
		Vector3 normalizedVector = Normalized();
		x = normalizedVector.x;
		y = normalizedVector.y;
		z = normalizedVector.z;
		return normalizedVector;
	}
	inline float Dot(const Vector3& other) const noexcept { return glm::dot(glm::vec3(x, y, z), glm::vec3(other)); }
	inline Vector3 Cross(const Vector3& other) const noexcept { return glm::cross(glm::vec3(x, y, z), glm::vec3(other)); }

	inline static float Dot(const Vector3& a, const Vector3& b) noexcept { return glm::dot(glm::vec3(a), glm::vec3(b)); }
	inline static Vector3 Cross(const Vector3& a, const Vector3& b) noexcept { return glm::cross(glm::vec3(a), glm::vec3(b)); }


};

}


#endif // ACRO_MATH_VECTOR3_H
