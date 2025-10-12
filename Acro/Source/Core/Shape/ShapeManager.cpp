#include "Core/Shape/ShapeManager.h"
#include <iostream>

using namespace Acro::Core;
using namespace Acro::Math;

ShapeHandle ShapeManager::CreateBoxShape(const Vector3& extent, const Vector3& offset) noexcept
{
	ShapeHandle handle;
	uint32_t denseIndex;

	if (!m_FreeHandles.empty())
	{
		handle = m_FreeHandles.back();
		m_FreeHandles.pop_back();
		denseIndex = static_cast<uint32_t>(m_ShapeData.shapeTypes.size());
		m_Sparse[handle.index] = denseIndex;
		m_RefCount[handle.index] = 0;
		m_Generations[handle.index]++;
	}
	else
	{
		// create new handle
		handle.index = static_cast<uint32_t>(m_Sparse.size());
		handle.generation = 0;
		m_Sparse.push_back(static_cast<uint32_t>(m_ShapeData.shapeTypes.size()));
		m_RefCount.push_back(static_cast<uint32_t>(0));
		m_Generations.push_back(0);
		denseIndex = static_cast<uint32_t>(m_ShapeData.shapeTypes.size());
	}

	m_ShapeData.shapeTypes.push_back(ShapeType::Box);
	m_ShapeData.offsets.push_back(offset);
	m_ShapeData.extents.push_back(extent);
	m_ShapeData.radii.push_back(0.0f);


	m_DenseToHandle.push_back(handle.index);
	return { handle.index,m_Generations[handle.index] };
}

ShapeHandle ShapeManager::CreateSphereShape(float radius, const Vector3& offset) noexcept
{
	ShapeHandle handle;
	uint32_t denseIndex;

	if (!m_FreeHandles.empty())
	{
		handle = m_FreeHandles.back();
		m_FreeHandles.pop_back();
		denseIndex = static_cast<uint32_t>(m_ShapeData.shapeTypes.size());
		m_Sparse[handle.index] = denseIndex;
		m_RefCount[handle.index] = 0;
		m_Generations[handle.index]++;
	}
	else
	{
		// create new handle
		handle.index = static_cast<uint32_t>(m_Sparse.size());
		handle.generation = 0;
		m_Sparse.push_back(static_cast<uint32_t>(m_ShapeData.shapeTypes.size()));
		m_RefCount.push_back(static_cast<uint32_t>(0));
		m_Generations.push_back(0);
		denseIndex = static_cast<uint32_t>(m_ShapeData.shapeTypes.size());
	}

	m_ShapeData.shapeTypes.push_back(ShapeType::Sphere);
	m_ShapeData.offsets.push_back(offset);
	m_ShapeData.extents.push_back(Vector3(0.0f));
	m_ShapeData.radii.push_back(radius);


	m_DenseToHandle.push_back(handle.index);
	return { handle.index,m_Generations[handle.index] };
}

void ShapeManager::AddRef(const ShapeHandle& handle)
{
	if (!IsValid(handle)) return;

	m_RefCount[handle.index]++;
}

void ShapeManager::ReleaseRef(const ShapeHandle& handle)
{
	if (!IsValid(handle)) return;
	if (m_RefCount[handle.index] == 0) return;

	m_RefCount[handle.index]--;

	if (m_RefCount[handle.index] <= 0)
	{
		DestroyShape(handle);
	}

}

void ShapeManager::DestroyShape(const ShapeHandle& handle) noexcept
{
	if (!IsValid(handle)) return;

	uint32_t denseIndex = m_Sparse[handle.index];
	uint32_t lastDense = static_cast<uint32_t>(m_ShapeData.shapeTypes.size() - 1);

	if (denseIndex != lastDense)
	{
		// Swap dense data
		m_ShapeData.shapeTypes[denseIndex] = m_ShapeData.shapeTypes[lastDense];
		m_ShapeData.offsets[denseIndex] = m_ShapeData.offsets[lastDense];
		m_ShapeData.extents[denseIndex] = m_ShapeData.extents[lastDense];
		m_ShapeData.radii[denseIndex] = m_ShapeData.radii[lastDense];

		// Update sparse 
		uint32_t lastHandleIndex = m_DenseToHandle[lastDense];
		m_Sparse[lastHandleIndex] = denseIndex;
		m_DenseToHandle[denseIndex] = lastHandleIndex;

	}

	// Pop back dense data
	m_ShapeData.shapeTypes.pop_back();
	m_ShapeData.offsets.pop_back();
	m_ShapeData.extents.pop_back();
	m_ShapeData.radii.pop_back();
	m_DenseToHandle.pop_back();
	m_RefCount[handle.index] = 0;
	m_Generations[handle.index]++;
	m_FreeHandles.push_back(handle);
}
