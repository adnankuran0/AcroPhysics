#ifndef ACRO_MATH_VECTOR3
#define ACRO_MATH_VECTOR3

#include "glm/glm.hpp"

namespace Acro::Math {
	
class Vector3
{

public:
	float x, y, z;
	Vector3() : x(0.0f) , y(0.0f), z(0.0f) {}
	Vector3(float x, float y, float z) : x(x), y(y), z(z) {}
	Vector3(const glm::vec3& v) : x(v.x) , y(v.y), z(v.z) {}


	operator glm::vec3() const { return glm::vec3(x, y, z); }

	Vector3 operator+(const Vector3& other) const { return Vector3(x + other.x, y + other.y, z + other.z); }
	Vector3& operator+=(const Vector3& other) 
	{
		x += other.x;
		y += other.y;
		z += other.z;
		return *this;
	}
	Vector3 operator-(const Vector3& other) const { return Vector3(x - other.x, y - other.y, z - other.z); }
	Vector3 operator*(float value) const { return Vector3(x * value, y * value, z * value); }


	inline float Length() const noexcept { return glm::length(glm::vec3(x, y, z)); }
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


#endif // ACRO_MATH_VECTOR3
