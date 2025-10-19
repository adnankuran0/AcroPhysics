#ifndef ACRO_BROADPHASE_H
#define ACRO_BROADPHASE_H

#include "Physics/Shape/ShapeInstanceManager.h"

namespace Acro::Physics {

struct Endpoint
{
	float value;
	uint32_t index;
	bool isMin;
};

class Broadphase
{
public:
	std::vector<std::pair<Acro::Physics::ShapeInstanceHandle, Acro::Physics::ShapeInstanceHandle>> Compute(Acro::Physics::ShapeInstanceManager* shapeInsanceManager);
};

}

#endif // !ACRO_BROADPHASE_H
