#ifndef ACRO_WORLD_DATA_H
#define ACRO_WORLD_DATA_H

#include "Math/Vector3.h"
#include <vector>

using namespace Acro::Math;

namespace Acro {

struct WorldData
{
	unsigned int bodyCount = 0;
	std::vector<Vector3> positions;
	std::vector<Vector3> velocities;
	std::vector<Vector3> accels;
};


}

#endif // !ACRO_WORLD_DATA_H
