#ifndef ACRO_SHAPE_INSTANCE_DATA_H
#define ACRO_SHAPE_INSTANCE_DATA_H

#include <vector>
#include "Physics/Shape/ShapeManager.h"
#include "Physics/Body/BodyManager.h"
#include "Math/Matrix4.h"
#include "Math/AABB.h"
#include "Public/CollisionLayer.h"

namespace Acro::Physics {

struct ShapeInstanceData
{
	std::vector<Physics::ShapeHandle> shapes;
	std::vector<Physics::BodyHandle> bodies;
	std::vector<Math::Matrix4> worldTransforms;
	std::vector<Math::AABB> worldAABBs;
	std::vector<CollisionFilter> filters;

	// flattened
	std::vector<Math::Vector3> worldVertexBuffer;
	std::vector<size_t> vertexOffsets;
	std::vector<size_t> vertexCounts;

};

}


#endif // !ACRO_SHAPE_INSTANCE_DATA_H
