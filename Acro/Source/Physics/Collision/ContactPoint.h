#ifndef ACRO_CONTACT_POINT_H
#define ACRO_CONTACT_POINT_H

#include "Math/Vector3.h"


namespace Acro::Physics {

struct ContactPoint
{
	Acro::Math::Vector3 position; // world position
	Acro::Math::Vector3 normal; // A -> B
	float penetration;
};

}

#endif // !ACRO_CONTACT_POINT_H
