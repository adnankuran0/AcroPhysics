#include "Physics/Shape/ShapeManager.h"
#include <iostream>

using namespace Acro::Physics;
using namespace Acro::Math;

ShapeHandle ShapeManager::CreateBoxShape(const Vector3& extent, const Vector3& offset) noexcept
{
	ShapeHandle handle;
	uint32_t denseIndex;

	CreateHandle(handle, denseIndex);

	m_ShapeData.shapeTypes.push_back(ShapeType::Box);
	m_ShapeData.offsets.push_back(offset);
	m_ShapeData.extents.push_back(extent);
	m_ShapeData.radii.push_back(0.0f);
	m_ShapeData.dirtyFlags.push_back(1);

	m_DenseToHandle.push_back(handle.index);
	return { handle.index,m_Generations[handle.index] };
}

ShapeHandle ShapeManager::CreateSphereShape(float radius, const Vector3& offset) noexcept
{
	ShapeHandle handle;
	uint32_t denseIndex;

	CreateHandle(handle, denseIndex);

	m_ShapeData.shapeTypes.push_back(ShapeType::Sphere);
	m_ShapeData.offsets.push_back(offset);
	m_ShapeData.extents.push_back(Vector3(0.0f));
	m_ShapeData.radii.push_back(radius);
	m_ShapeData.dirtyFlags.push_back(1);

	m_DenseToHandle.push_back(handle.index);
	return { handle.index,m_Generations[handle.index] };
}


void ShapeManager::DestroyShape(const ShapeHandle& handle) noexcept
{
	if (!IsValid(handle)) return;

	uint32_t denseIndex = m_Sparse[handle.index];
	uint32_t lastDense = static_cast<uint32_t>(m_ShapeData.shapeTypes.size() - 1);

	
	SwapDenseData(lastDense, denseIndex);

	DestroyHandle(handle, lastDense, denseIndex);

	PopBackDenseData();

	
}


void ShapeManager::SwapDenseData(size_t from, size_t to) noexcept
{
	m_ShapeData.shapeTypes[to] = m_ShapeData.shapeTypes[from];
	m_ShapeData.offsets[to] = m_ShapeData.offsets[from];
	m_ShapeData.extents[to] = m_ShapeData.extents[from];
	m_ShapeData.radii[to] = m_ShapeData.radii[from];
	m_ShapeData.dirtyFlags[to] = m_ShapeData.dirtyFlags[from];
}

void ShapeManager::PopBackDenseData() noexcept
{
	m_ShapeData.shapeTypes.pop_back();
	m_ShapeData.offsets.pop_back();
	m_ShapeData.extents.pop_back();
	m_ShapeData.radii.pop_back();
	m_ShapeData.dirtyFlags.pop_back();
	m_DenseToHandle.pop_back();
}