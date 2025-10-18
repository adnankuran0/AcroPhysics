#ifndef ACRO_BROADPHASE_H
#define ACRO_BROADPHASE_H

#include "Shape/ShapeInstanceManager.h"

namespace Acro::Core {

struct Endpoint
{
	float value;
	uint32_t index;
	bool isMin;
};

class Broadphase
{
public:
	std::vector<std::pair<Acro::Core::ShapeInstanceHandle, Acro::Core::ShapeInstanceHandle>> Compute(Acro::Core::ShapeInstanceManager* shapeInsanceManager);
};

}

#endif // !ACRO_BROADPHASE_H
