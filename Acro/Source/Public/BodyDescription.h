#ifndef ACRO_BODY_DESCRIPTION_H
#define ACRO_BODY_DESCRIPTION_H

#include "Math/Vector3.h"
#include "Math/Quaternion.h"

namespace Acro {

	struct BodyDescription
	{
		Acro::Math::Vector3 position = Acro::Math::Vector3(0.0f);
		Acro::Math::Vector3 linearVelocity = Acro::Math::Vector3(0.0f);
		Acro::Math::Quaternion orientation = Acro::Math::Quaternion::Identity();
		Acro::Math::Vector3 angularVelocity = Acro::Math::Vector3(0.0f);
		Acro::Math::Vector3 forceAccumulation = Acro::Math::Vector3(0.0f);
		float mass = 1.0f;
	};
}

#endif // ACRO_BODY_DESCRIPTION_H