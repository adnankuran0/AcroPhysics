#ifndef ACRO_MATH_VEC3_H
#define ACRO_MATH_VEC3_H

#include <cmath>
#include <cstdint>
#include <limits>
#include <functional>
#include <type_traits>
#include <bit>

namespace acro::math {

struct Vec3
{
	float x, y, z;

	constexpr Vec3() noexcept : x(0), y(0), z(0) {}
	constexpr Vec3(float x_, float y_, float z_) noexcept : x(x_), y(y_),z(z_) {}
	constexpr Vec3(float v) noexcept : x(v), y(v), z(v) {}

	static constexpr Vec3 zero() noexcept { return Vec3(0.0f); }
	static constexpr Vec3 unitX() noexcept { return Vec3(1, 0, 0); }
	static constexpr Vec3 unitY() noexcept { return Vec3(0, 1, 0); }
	static constexpr Vec3 unitZ() noexcept { return Vec3(0, 0, 1); }
};

static_assert(std::is_trivially_copyable_v<Vec3>, "Vec3 must be trivially copyable");
static_assert(std::is_standard_layout_v<Vec3>, "Vec3 must be standard layout");
static_assert(sizeof(Vec3) == 12, "Vec3 must be 12 bytes");

[[nodiscard]] constexpr Vec3 operator+(Vec3 a, Vec3 b) noexcept { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
[[nodiscard]] constexpr Vec3 operator-(Vec3 a, Vec3 b) noexcept { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
[[nodiscard]] constexpr Vec3 operator-(Vec3 a) noexcept { return { -a.x, -a.y, -a.z }; }

[[nodiscard]] constexpr Vec3 operator*(Vec3 a, float s) noexcept { return { a.x * s, a.y * s, a.z * s }; }
[[nodiscard]] constexpr Vec3 operator*(float s, Vec3 a) noexcept { return a * s; }
[[nodiscard]] constexpr Vec3 operator/(Vec3 a, float s) noexcept { return { a.x / s, a.y / s, a.z / s }; }

inline Vec3& operator+=(Vec3& a, Vec3 b) noexcept { a.x += b.x; a.y += b.y; a.z += b.z; return a; }
inline Vec3& operator-=(Vec3& a, Vec3 b) noexcept { a.x -= b.x; a.y -= b.y; a.z -= b.z; return a; }
inline Vec3& operator*=(Vec3& a, float s) noexcept { a.x *= s; a.y *= s; a.z *= s; return a; }
inline Vec3& operator/=(Vec3& a, float s) noexcept { a.x /= s; a.y /= s; a.z /= s; return a; }

[[nodiscard]] constexpr float Dot(Vec3 a, Vec3 b) noexcept { return a.x * b.x + a.y * b.y + a.z * b.z; }

[[nodiscard]] constexpr Vec3 Cross(Vec3 a, Vec3 b) noexcept {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

[[nodiscard]] constexpr float LengthSquared(Vec3 a) noexcept { return dot(a, a); }

[[nodiscard]] inline float Length(Vec3 a) noexcept {
    return std::sqrt(LengthSquared(a));
}

[[nodiscard]] inline Vec3 Normalize(Vec3 a, float eps = 1e-8f) noexcept {
    float l2 = LengthSquared(a);
    if (l2 <= eps || !std::isfinite(l2)) return Vec3::zero();
    float invL = 1.0f / std::sqrt(l2);
    return a * invL;
}

[[nodiscard]] inline bool ApproxEqual(Vec3 a, Vec3 b, float eps = 1e-5f) noexcept {
    return std::fabs(a.x - b.x) <= eps && std::fabs(a.y - b.y) <= eps && std::fabs(a.z - b.z) <= eps;
}

[[nodiscard]] constexpr Vec3 Lerp(Vec3 a, Vec3 b, float t) noexcept { return a + (b - a) * t; }

struct Vec3Hash {
    size_t operator()(const Vec3& v) const noexcept {
        auto h = std::hash<uint32_t>{};
        uint32_t ix = std::bit_cast<uint32_t>(v.x);
        uint32_t iy = std::bit_cast<uint32_t>(v.y);
        uint32_t iz = std::bit_cast<uint32_t>(v.z);
        size_t seed = h(ix) + 0x9e3779b97f4a7c15ULL;
        seed ^= h(iy) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
        seed ^= h(iz) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
        return seed;
    }
};

} // namespace acro::math 

#endif // !ACRO_MATH_VEC3_H
