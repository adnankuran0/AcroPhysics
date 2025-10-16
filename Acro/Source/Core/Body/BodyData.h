#ifndef ACRO_BODY_DATA_H
#define ACRO_BODY_DATA_H

#include <vector>
#include "Math/Vector3.h"
#include "Math/Quaternion.h"
#include "Core/Shape/ShapeManager.h"

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


namespace Acro::Core {

struct ShapeHandle; // forward declaration

struct BodyData
{
	std::vector<Acro::Math::Vector3> positions;
	std::vector<Acro::Math::Vector3> linearVelocities;
	std::vector<Acro::Math::Quaternion> orientations;
	std::vector<Acro::Math::Vector3> angularVelocities;
	std::vector<Acro::Math::Vector3> forceAccumulators;
	std::vector<float> inverseMasses;

	std::vector<Acro::Core::ShapeHandle> shapes;
	std::vector<size_t> shapesStart;
	std::vector<size_t> shapesCount;


	void Reserve(size_t capacity);
	
};



}

#endif // !ACRO_BODY_DATA_H
