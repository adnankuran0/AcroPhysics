#ifndef ACRO_SHAPE_INSTANCE_DATA_H
#define ACRO_SHAPE_INSTANCE_DATA_H

#include <vector>
#include "Core/Shape/ShapeManager.h"
#include "Core/Body/BodyManager.h"
#include "Math/Matrix4.h"
#include "Math/AABB.h"


namespace Acro::Core {

struct ShapeInstanceData
{
	std::vector<Core::ShapeHandle> shapes;
	std::vector<Core::BodyHandle> bodies;
	std::vector<Math::Matrix4> worldTransforms;
	std::vector<Math::AABB> worldAABBs;
	std::vector<uint8_t> dirtyFlags;

	// flattened
	std::vector<Math::Vector3> worldVertexBuffer;
	std::vector<size_t> vertexOffsets;
	std::vector<size_t> vertexCounts;

};

}


#endif // !ACRO_SHAPE_INSTANCE_DATA_H
