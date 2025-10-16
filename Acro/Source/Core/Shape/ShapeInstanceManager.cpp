#include "ShapeInstanceManager.h"
#include "Math/Matrix4.h"
#include "Debug/DebugRenderer.h"
#include <iostream>


using namespace Acro::Core;
using namespace Acro::Math;


ShapeInstanceHandle Acro::Core::ShapeInstanceManager::CreateShapeInstance(const ShapeHandle& shapeHandle, const BodyHandle& bodyHandle)
{
	ShapeInstanceHandle handle;
	uint32_t denseIndex;

	CreateHandle(handle, denseIndex);

	m_ShapeInstanceData.shapes.push_back(shapeHandle); 
	m_ShapeInstanceData.bodies.push_back(bodyHandle);
	m_ShapeInstanceData.worldTransforms.push_back(Matrix4(1.0f));
	m_ShapeInstanceData.worldAABBs.push_back(AABB());
	m_ShapeInstanceData.dirtyFlags.push_back(1);
	
	m_DenseToHandle.push_back(handle.index);

	return { handle.index,m_Generations[handle.index] };
}

void Acro::Core::ShapeInstanceManager::DestroyShapeInstance(const ShapeInstanceHandle& handle)
{
	if (!IsValid(handle)) return;

	uint32_t denseIndex = m_Sparse[handle.index];
	uint32_t lastDense = static_cast<uint32_t>(m_ShapeInstanceData.shapes.size() - 1);

	if (denseIndex != lastDense)
	{
		// Swap dense data
		m_ShapeInstanceData.shapes[denseIndex] = m_ShapeInstanceData.shapes[lastDense];
		m_ShapeInstanceData.bodies[denseIndex] = m_ShapeInstanceData.bodies[lastDense];
		m_ShapeInstanceData.worldTransforms[denseIndex] = m_ShapeInstanceData.worldTransforms[lastDense];
		m_ShapeInstanceData.worldAABBs[denseIndex] = m_ShapeInstanceData.worldAABBs[lastDense];
		m_ShapeInstanceData.dirtyFlags[denseIndex] = m_ShapeInstanceData.dirtyFlags[lastDense];

		// Update sparse 
		uint32_t lastHandleIndex = m_DenseToHandle[lastDense];
		m_Sparse[lastHandleIndex] = denseIndex;
		m_DenseToHandle[denseIndex] = lastHandleIndex;

	}
	
	
	// Pop back dense data
	m_ShapeInstanceData.shapes.pop_back();
	m_ShapeInstanceData.bodies.pop_back();
	m_ShapeInstanceData.worldTransforms.pop_back();
	m_ShapeInstanceData.worldAABBs.pop_back();
	m_ShapeInstanceData.dirtyFlags.pop_back();
	m_DenseToHandle.pop_back();
	m_Generations[handle.index]++;
	m_FreeHandles.push_back(handle);

	m_FreeHandles.push_back(handle);

}

void ShapeInstanceManager::UpdateWorldData(Acro::Core::BodyManager& bodyManager, Acro::Core::ShapeManager& shapeManager)
{
	auto& data = m_ShapeInstanceData;

	for (size_t i = 0; i < data.shapes.size(); i++)
	{
		if (!data.dirtyFlags[i]) continue;

		const BodyHandle& body = data.bodies[i];
		const ShapeHandle& shape = data.shapes[i];
		if (shape == ShapeHandle::Null()) continue;

		Vector3 pos = bodyManager.GetPosition(body);
		Quaternion rot = bodyManager.GetOrientation(body);

		Vector3 offset = shapeManager.GetOffset(shape);

		data.worldTransforms[i] =  Matrix4::Translation(pos + rot * offset) * rot.ToMat4();

		if (shapeManager.GetShapeType(shape) == ShapeType::Box)
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
			Vector3 worldVerts[8];
			for (int v = 0; v < 8; v++)
				worldVerts[v] = worldTransform * localVerts[v];

			
			data.worldAABBs[i] = AABB::FromVertices(worldVerts, 8);
			auto& aabb = data.worldAABBs[i];
			Vector3 color = Vector3(1.0f, 0.0f, 0.0f);

		}

		//data.dirtyFlags[i] = 0;
	}
}

void Acro::Core::ShapeInstanceManager::CreateHandle(ShapeInstanceHandle& outHandle, uint32_t& outDenseIndex) noexcept
{
	if (!m_FreeHandles.empty())
	{
		outHandle = m_FreeHandles.back();
		m_FreeHandles.pop_back();
		outDenseIndex = static_cast<uint32_t>(m_ShapeInstanceData.shapes.size());
		m_Sparse[outHandle.index] = outDenseIndex;
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
