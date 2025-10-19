#ifndef ACRO_BODY_DATA_H
#define ACRO_BODY_DATA_H

#include <vector>

#include "Math/Vector3.h"
#include "Math/Quaternion.h"
#include "Physics/Shape/ShapeManager.h"

namespace Acro::Physics {

struct BodyData
{
	std::vector<Acro::Math::Vector3> positions;
	std::vector<Acro::Math::Vector3> linearVelocities;
	std::vector<Acro::Math::Quaternion> orientations;
	std::vector<Acro::Math::Vector3> angularVelocities;
	std::vector<Acro::Math::Vector3> forceAccumulators;
	std::vector<float> inverseMasses;
	std::vector<uint8_t> dirtyFlags;

	std::vector<Acro::Physics::ShapeHandle> shapeBuffer;
	std::vector<size_t> shapeOffsets;
	std::vector<size_t> shapeCounts;

	void Reserve(size_t capacity);
	
};

}

#endif // !ACRO_BODY_DATA_H
