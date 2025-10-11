#ifndef ACRO_BODY_DATA_H
#define ACRO_BODY_DATA_H

#include "Math/Vector3.h"
#include "Math/Quaternion.h"
#include <vector>

using namespace Acro::Math;

namespace Acro::Core {

struct BodyData
{
	std::vector<Vector3> positions;
	std::vector<Vector3> linearVelocities;
	std::vector<Quaternion> orientations;
	std::vector<Vector3> angularVelocities;
	std::vector<Vector3> forceAccumulators;
	std::vector<float> masses;
};

}

#endif // !ACRO_BODY_DATA_H
