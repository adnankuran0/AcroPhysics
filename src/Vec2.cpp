#include "Vec2.h"

namespace acro {

	Vec2::Vec2(float x, float y) : x(x), y(y) {}

	Vec2 Vec2::zero = Vec2(0, 0);

	Vec2 Vec2::operator+(const Vec2& other) const
	{
		return Vec2(x + other.x, y + other.y);
	}

	Vec2& Vec2::operator+=(const Vec2& other)
	{
		x += other.x;
		y += other.y;
		return *this;
	}

	Vec2 Vec2::operator-(const Vec2& other) const
	{
		return Vec2(x - other.x, y - other.y);
	}

	Vec2& Vec2::operator-=(const Vec2& other)
	{
		x -= other.x;
		y -= other.y;
		return *this;
	}

	Vec2 Vec2::operator*(const Vec2& other) const
	{
		return Vec2(x * other.x, y * other.y);
	}
	
	Vec2 Vec2::operator*(const float other) const
	{
		return Vec2(x * other, y * other);
	}

	Vec2& Vec2::operator*=(const Vec2& other) 
	{
		x *= other.x;
		y *= other.y;
		return *this;
	}

	Vec2& Vec2::operator*=(const float other)
	{
		x *= other;
		y *= other;
		return *this;
	}

	Vec2 Vec2::operator/(const Vec2& other) const
	{
		if (other.x == 0 || other.y == 0) throw std::invalid_argument("Division by zero");
		return Vec2(x / other.x, y / other.y);
	}

	Vec2 Vec2::operator/(const float other) const
	{
		if (other == 0) throw std::invalid_argument("Division by zero");
		return Vec2(x / other, y / other);
	}



	Vec2& Vec2::operator/=(const Vec2& other)
	{
		if (other.x == 0 || other.y == 0) throw std::invalid_argument("Division by zero");
		x /= other.x;
		y /= other.y;
		return *this;
	}

	Vec2& Vec2::operator/=(const float other)
	{
		if (other == 0) throw std::invalid_argument("Division by zero");
		x /= other;
		y /= other;
		return *this;
	}

	bool Vec2::operator==(const Vec2& other) const
	{
		
		return fabs(x - other.x) < Math::epsilon && fabs(y - other.y) < Math::epsilon;
	}

	bool Vec2::operator!=(const Vec2& other) const
	{
		return !(*this == other);
	}

	Vec2 Vec2::operator-() const
	{
		return Vec2(-x, -y);
	}

	float Vec2::magnitude() const
	{
		return sqrt(x*x + y*y);
	}

	float Vec2::magnitudeSquared() const
	{
		return x * x + y * y;
	}

	Vec2 Vec2::normalized() const
	{
		float magSquared = magnitudeSquared();
		if (magSquared < 1e-10f) return Vec2(0, 0);
		float invMag = 1.0f / sqrt(magSquared);
		return Vec2(x * invMag, y * invMag);
	}

	float Vec2::dot(const Vec2& other) const
	{
		return x * other.x + y * other.y;
	}

	float Vec2::cross(const Vec2& other) const
	{
		return x * other.y - y * other.x;
	}

	float Vec2::angleBetween(const Vec2& other) const
	{
		float dotProd = dot(other);
		float mags = magnitude() + other.magnitude();
		return acos(dotProd / mags);
	}

	float Vec2::distance(const Vec2& other) const
	{
		return sqrt((other.x - x) * (other.x - x) + (other.y - y) * (other.y - y));
	}

	float* Vec2::getArray() const
	{
		static float arr[2] = { x,y };
		return arr;
	}

	Vec2 Vec2::clamp(const Vec2& min, const Vec2& max) const
	{
		return Vec2(
			std::max(min.x, std::min(x, max.x)),
			std::max(min.y, std::min(y, max.y))
		);
	}

	bool Vec2::nearlyEquals(const Vec2& a, const Vec2& b) 
	{
		return (b-a).magnitudeSquared() < Math::epsilon * Math::epsilon;
	}

}