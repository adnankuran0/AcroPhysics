#include "ShapeInstanceManager.h"
#include "Math/Matrix4.h"
#include "Debug/DebugRenderer.h"
#include <iostream>


using namespace Acro::Core;
using namespace Acro::Math;


ShapeInstanceHandle Acro::Core::ShapeInstanceManager::CreateShapeInstance(ShapeManager& shapeManager, const ShapeHandle& shapeHandle, const BodyHandle& bodyHandle)
{
	ShapeInstanceHandle handle;
	uint32_t denseIndex;

	CreateHandle(handle, denseIndex);

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
		}
	}

	size_t offset = data.worldVertexBuffer.size();
	data.vertexOffsets.push_back(offset);
	data.vertexCounts.push_back(vertexCount);

	data.worldVertexBuffer.resize(offset + vertexCount);
	
	m_DenseToHandle.push_back(handle.index);

	return { handle.index,m_Generations[handle.index] };
}

void Acro::Core::ShapeInstanceManager::DestroyShapeInstance(const ShapeInstanceHandle& handle)
{
	if (!IsValid(handle)) return;

	uint32_t denseIndex = m_Sparse[handle.index];
	uint32_t lastDense = static_cast<uint32_t>(m_ShapeInstanceData.shapes.size() - 1);

	auto& data = m_ShapeInstanceData;

	size_t offset = data.vertexOffsets[denseIndex];
	size_t count = data.vertexCounts[denseIndex];
	size_t lastOffset = data.vertexOffsets[lastDense];
	size_t lastCount = data.vertexCounts[lastDense];

	if (denseIndex != lastDense)
	{
		// Swap dense data
		data.shapes[denseIndex] =			data.shapes[lastDense];
		data.bodies[denseIndex] =			data.bodies[lastDense];
		data.worldTransforms[denseIndex] =  data.worldTransforms[lastDense];
		data.worldAABBs[denseIndex] =		data.worldAABBs[lastDense];
		data.filters[denseIndex] =			data.filters[lastDense];

		// swap vertex buffer
		for (size_t v = 0; v < lastCount; v++)
		{
			data.worldVertexBuffer[offset + v] = data.worldVertexBuffer[lastOffset + v];
		}
		data.vertexOffsets[denseIndex] = offset;
		data.vertexCounts[denseIndex] = lastCount;

		// Update sparse 
		uint32_t lastHandleIndex = m_DenseToHandle[lastDense];
		m_Sparse[lastHandleIndex] = denseIndex;
		m_DenseToHandle[denseIndex] = lastHandleIndex;

	}
	
	
	// Pop back dense data
	data.shapes.pop_back();
	data.bodies.pop_back();
	data.worldTransforms.pop_back();
	data.worldAABBs.pop_back();
	data.filters.pop_back();

	data.vertexOffsets.pop_back();
	data.vertexCounts.pop_back();
	data.worldVertexBuffer.resize(data.worldVertexBuffer.size() - count);

	m_DenseToHandle.pop_back();
	m_Generations[handle.index]++;
	m_FreeHandles.push_back(handle);


}

void ShapeInstanceManager::UpdateWorldData(Acro::Core::BodyManager& bodyManager, Acro::Core::ShapeManager& shapeManager)
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
		}

		bodyManager.SetDirty(body, 0);
		shapeManager.SetDirty(shape, 0);

	}
}

void Acro::Core::ShapeInstanceManager::CreateHandle(ShapeInstanceHandle& outHandle, uint32_t& outDenseIndex) noexcept
{
	if (!m_FreeHandles.empty())
	{
		outHandle = m_FreeHandles.back();
		m_FreeHandles.pop_back();
		outDenseIndex = m_Sparse[outHandle.index]; 
		m_Generations[outHandle.index]++;
	}
	else
	{
		// create new handle
		outHandle.index = static_cast<uint32_t>(m_Sparse.size());
		outHandle.generation = 0;
		m_Sparse.push_back(static_cast<uint32_t>(m_ShapeInstanceData.shapes.size()));
		m_Generations.push_back(0);
		outDenseIndex = static_cast<uint32_t>(m_ShapeInstanceData.shapes.size());
	}
}
