#ifndef ACRO_BODY_DATA_H
#define ACRO_BODY_DATA_H

#include <vector>
#include "Math/Vector3.h"
#include "Math/Quaternion.h"

namespace Acro::Core {

struct ShapeHandle; // forward declaration

struct BodyData
{
	std::vector<Acro::Math::Vector3> positions;
	std::vector<Acro::Math::Vector3> linearVelocities;
	std::vector<Acro::Math::Quaternion> orientations;
	std::vector<Acro::Math::Vector3> angularVelocities;
	std::vector<Acro::Math::Vector3> forceAccumulators;
	std::vector<float> masses;
	std::vector<Acro::Core::ShapeHandle> shapes;
};

}

#endif // !ACRO_BODY_DATA_H
