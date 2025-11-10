#include "ShapeInstanceManager.h"
#include "Math/Matrix4.h"
#include "Debug/DebugRenderer.h"
#include <iostream>


using namespace Acro::Math;
using namespace Acro::Physics;


ShapeInstanceHandle ShapeInstanceManager::CreateShapeInstance(ShapeManager& shapeManager, const ShapeHandle& shapeHandle, const BodyHandle& bodyHandle)
{
	ShapeInstanceHandle handle;
	uint32_t to;

	CreateHandle(handle, to);

	auto& data = m_ShapeInstanceData;
	data.shapes.push_back(shapeHandle); 
	data.bodies.push_back(bodyHandle);
	data.worldTransforms.push_back(Matrix4(1.0f));
	data.worldAABBs.push_back(AABB());
	data.filters.push_back(CollisionFilter{});

	// vertex cache
	size_t vertexCount = 0;
	if (shapeHandle != ShapeHandle::Null())
	{
		switch (shapeManager.GetShapeType(shapeHandle))
		{
		case ShapeType::Box:
			vertexCount = 8;
			break;
    case ShapeType::Null:
      vertexCount = 0;
      break;
    case ShapeType::Sphere:
      vertexCount = 0;
      break;
    default:
      vertexCount = 0;
      break;
		}
	}

	size_t offset = data.worldVertexBuffer.size();
	data.vertexOffsets.push_back(offset);
	data.vertexCounts.push_back(vertexCount);

	data.worldVertexBuffer.resize(offset + vertexCount);
	
	m_DenseToHandle.push_back(handle.index);

	return { handle.index,m_Generations[handle.index] };
}

void ShapeInstanceManager::DestroyShapeInstance(const ShapeInstanceHandle& handle)
{
	if (!IsValid(handle)) return;

	uint32_t to = m_Sparse[handle.index];
	uint32_t from = static_cast<uint32_t>(m_ShapeInstanceData.shapes.size() - 1);

	auto& data = m_ShapeInstanceData;

	size_t offset = data.vertexOffsets[to];
	size_t count = data.vertexCounts[to];
	size_t lastOffset = data.vertexOffsets[from];
	size_t lastCount = data.vertexCounts[from];

	SwapDenseData(lastCount, to);

	if (to != from)
	{

		// swap vertex buffer
		for (size_t v = 0; v < lastCount; v++)
		{
			data.worldVertexBuffer[offset + v] = data.worldVertexBuffer[lastOffset + v];
		}
		data.vertexOffsets[to] = offset;
		data.vertexCounts[to] = lastCount;

		DestroyHandle(handle,from,to);

	}
	
	PopBackDenseData(count);



}

void ShapeInstanceManager::UpdateWorldData(BodyManager& bodyManager, ShapeManager& shapeManager)
{
	auto& data = m_ShapeInstanceData;

	for (size_t i = 0; i < data.shapes.size(); i++)
	{
		const BodyHandle& body = data.bodies[i];
		const ShapeHandle& shape = data.shapes[i];

		if (!bodyManager.IsDirty(body)
			&& !shapeManager.IsDirty(shape)) continue;

		
		if (shape == ShapeHandle::Null()) continue;

		Vector3 pos = bodyManager.GetPosition(body);
		Quaternion rot = bodyManager.GetOrientation(body);

		Vector3 offset = shapeManager.GetOffset(shape);

		data.worldTransforms[i] =  Matrix4::Translation(pos + rot * offset) * rot.ToMat4();

		Vector3* verts = GetWorldVertices(i);
		size_t count = GetWorldVertexCount(i);

		switch (shapeManager.GetShapeType(shape))
		{
		case ShapeType::Box:
		{
			Vector3 extent = shapeManager.GetExtent(shape);

			Vector3 localVerts[8] = {
			{-extent.x, -extent.y, -extent.z},
			{ extent.x, -extent.y, -extent.z},
			{ extent.x,  extent.y, -extent.z},
			{-extent.x,  extent.y, -extent.z},
			{-extent.x, -extent.y,  extent.z},
			{ extent.x, -extent.y,  extent.z},
			{ extent.x,  extent.y,  extent.z},
			{-extent.x,  extent.y,  extent.z},
			};

			Matrix4 worldTransform = Matrix4::Translation(pos + rot * offset) * rot.ToMat4();
			for (int v = 0; v < count; v++)
				verts[v] = worldTransform * localVerts[v];

			data.worldAABBs[i] = AABB::FromVertices(verts, 8);
			break;
		}
		case ShapeType::Sphere:
		{
			float radius = shapeManager.GetRadius(shape);

			Vector3 min = (pos + offset) - radius;
			Vector3 max = (pos + offset) + radius;

			data.worldAABBs[i] = AABB(min, max);
			break;
		}
    case ShapeType::Null:
      break;
		}

		bodyManager.SetDirty(body, 0);
		shapeManager.SetDirty(shape, 0);

	}
}



void ShapeInstanceManager::SwapDenseData(size_t from, size_t to) noexcept
{
	m_ShapeInstanceData.shapes[to] = m_ShapeInstanceData.shapes[from];
	m_ShapeInstanceData.bodies[to] = m_ShapeInstanceData.bodies[from];
	m_ShapeInstanceData.worldTransforms[to] = m_ShapeInstanceData.worldTransforms[from];
	m_ShapeInstanceData.worldAABBs[to] = m_ShapeInstanceData.worldAABBs[from];
	m_ShapeInstanceData.filters[to] = m_ShapeInstanceData.filters[from];
}

void ShapeInstanceManager::PopBackDenseData(size_t vertexCount) noexcept
{
	auto& data = m_ShapeInstanceData;
	data.shapes.pop_back();
	data.bodies.pop_back();
	data.worldTransforms.pop_back();
	data.worldAABBs.pop_back();
	data.filters.pop_back();
	data.vertexOffsets.pop_back();
	data.vertexCounts.pop_back();
	data.worldVertexBuffer.resize(data.worldVertexBuffer.size() - vertexCount);
	m_DenseToHandle.pop_back();
}
