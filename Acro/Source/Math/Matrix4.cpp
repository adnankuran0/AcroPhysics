#include "Math/Matrix4.h"

using namespace Acro::Math;

void Matrix4::Decompose(Vector3& outTranslation, Quaternion& outRotation, Vector3& outScale) const noexcept
{
	glm::vec3 t, s, skew;
	glm::vec4 persp;
	glm::quat r;
	glm::decompose(m, s, r, t, skew, persp);
	outTranslation = Vector3(t);
	outRotation = Quaternion(r);
	outScale = Vector3(s);
}