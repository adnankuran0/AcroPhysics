#include "Math/Quaternion.h"
#include "Math/Matrix4.h"

using namespace Acro::Math;

Matrix4 Quaternion::ToMat4() const noexcept
{
	return Matrix4(glm::toMat4(q)); 
}
