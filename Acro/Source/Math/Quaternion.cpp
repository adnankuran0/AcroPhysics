#include "Math/Quaternion.h"
#include "Math/Matrix4.h"

Acro::Math::Matrix4 Acro::Math::Quaternion::ToMat4() const noexcept
{
	return Acro::Math::Matrix4(glm::toMat4(q)); 
}
