#pragma once
#include <cmath>
#include <stdexcept>
#include "AcroMath.h"

namespace acro {

	class Vec2
	{
	public:
		float x, y;

		Vec2(float x = 0, float y = 0);
		static Vec2 zero; 

		Vec2 operator+(const Vec2& other) const;
		Vec2& operator+=(const Vec2& other);
		Vec2 operator-(const Vec2& other) const;
		Vec2& operator-=(const Vec2& other);
		Vec2 operator*(const Vec2& other) const;
		Vec2 operator*(const float other) const;
		Vec2& operator*=(const Vec2& other);
		Vec2& operator*=(const float other);
		Vec2 operator/(const Vec2& other) const;
		Vec2 operator/(const float other) const;
		Vec2& operator/=(const Vec2& other);
		Vec2& operator/=(const float other);
		Vec2 operator-() const;
		Vec2 normalized() const;
		bool operator==(const Vec2& other) const;
		bool operator!=(const Vec2& other) const;
		float magnitude() const;
		float magnitudeSquared() const;
		float dot(const Vec2& other) const;
		float cross(const Vec2& other) const;
		float angleBetween(const Vec2& other) const;
		float distance(const Vec2& other) const;
		float* getArray() const;
		Vec2 clamp(const Vec2& min, const Vec2& max) const;
		static bool nearlyEquals(const Vec2& a, const Vec2& b);

	};
}

