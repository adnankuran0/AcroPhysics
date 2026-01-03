#include <algorithm>

#include "Physics/Body/BodyManager.h"
#include "Physics/Shape/ShapeManager.h"
#include "Public/BodyDescription.h"


using namespace Acro::Math;
using namespace Acro::Physics;

BodyHandle BodyManager::CreateBody() noexcept
{
	BodyHandle handle;
	uint32_t denseIndex;

	CreateHandle(handle, denseIndex);

	BodyDescription desc;
	PushData(desc);

	m_DenseToHandle.push_back(handle.index);

	SetMass(handle, desc.mass);

	return { handle.index,m_Generations[handle.index] };

}

BodyHandle BodyManager::CreateBody(const BodyDescription& desc) noexcept
{
	BodyHandle handle;
	uint32_t denseIndex;

	CreateHandle(handle, denseIndex);

	PushData(desc);

	m_DenseToHandle.push_back(handle.index);

	SetMass(handle, desc.mass);

	return { handle.index,m_Generations[handle.index] };
}

void BodyManager::DestroyBody(const BodyHandle& handle) noexcept
{
	if (!IsValid(handle)) return;

	uint32_t denseIndex = m_Sparse[handle.index];
	uint32_t lastDense = static_cast<uint32_t>(m_BodyData.positions.size() - 1);

	SwapDenseData(lastDense, denseIndex);

	DestroyHandle(handle,lastDense,denseIndex);

	PopBackDenseData();


}

void BodyManager::AttachShape(const BodyHandle& bodyHandle, const ShapeHandle& shapeHandle)
{
	assert(IsValid(bodyHandle));

	size_t bodyIndex = m_Sparse[bodyHandle.index];

	if (m_BodyData.shapeCounts[bodyIndex] == 0)
	{
		m_BodyData.shapeOffsets[bodyIndex] = m_BodyData.shapeBuffer.size();
	}

	m_BodyData.shapeBuffer.push_back(shapeHandle);
	m_BodyData.shapeCounts[bodyIndex]++;
}

void BodyManager::DetachShape(const BodyHandle& bodyHandle, const ShapeHandle& shapeHandle)
{
	assert(IsValid(bodyHandle));
	size_t bodyIndex = m_Sparse[bodyHandle.index];

	size_t start = m_BodyData.shapeOffsets[bodyIndex];
	size_t count = m_BodyData.shapeCounts[bodyIndex];

	auto begin = m_BodyData.shapeBuffer.begin() + start;
	auto end = begin + count;
	auto it = std::find(begin, end, shapeHandle);

	if (it != end)
	{
		// swap remove
		*it = m_BodyData.shapeBuffer.back();
		m_BodyData.shapeBuffer.pop_back();
		m_BodyData.shapeCounts[bodyIndex]--;
	}
}

void BodyManager::DetachShape(const BodyHandle& bodyHandle)
{
	assert(IsValid(bodyHandle));

	// Detach all shapes attached to body

	size_t bodyIndex = m_Sparse[bodyHandle.index];

	size_t start = m_BodyData.shapeOffsets[bodyIndex];
	size_t count = m_BodyData.shapeCounts[bodyIndex];

	if (count == 0) return;

	m_BodyData.shapeBuffer.erase(m_BodyData.shapeBuffer.begin() + start, m_BodyData.shapeBuffer.begin() + start + count);

	m_BodyData.shapeCounts[bodyIndex] = 0;

	for (size_t i = bodyIndex + 1; i < m_BodyData.shapeOffsets.size(); ++i)
	{
		if (m_BodyData.shapeCounts[i] > 0)
			m_BodyData.shapeOffsets[i] -= count;
	}
}

bool BodyManager::HasShape(const BodyHandle& bodyHandle, const ShapeHandle& shapeHandle)
{
	assert(IsValid(bodyHandle));

	size_t bodyIndex = m_Sparse[bodyHandle.index];

	size_t start = m_BodyData.shapeOffsets[bodyIndex];
	size_t count = m_BodyData.shapeCounts[bodyIndex];

	auto begin = m_BodyData.shapeBuffer.begin() + start;
	auto end = begin + count;

	return std::find(begin, end, shapeHandle) != end;
}

bool BodyManager::HasShape(const BodyHandle& bodyHandle)
{
	assert(IsValid(bodyHandle));
	size_t bodyIndex = m_Sparse[bodyHandle.index];
	size_t count = m_BodyData.shapeCounts[bodyIndex];
	return count > 0;
}

void BodyManager::PushData(const Acro::BodyDescription& desc) noexcept
{
	m_BodyData.positions.push_back(desc.position);
	m_BodyData.linearVelocities.push_back(desc.linearVelocity);
	m_BodyData.orientations.push_back(desc.orientation);
	m_BodyData.angularVelocities.push_back(desc.angularVelocity);
	m_BodyData.forceAccumulators.push_back(desc.forceAccumulation);
	m_BodyData.inverseMasses.push_back(0);
	m_BodyData.dirtyFlags.push_back(1);
	m_BodyData.shapeCounts.push_back(0);
	m_BodyData.shapeOffsets.push_back(0);
}

void BodyManager::SwapDenseData(size_t from, size_t to) noexcept
{
	m_BodyData.positions[to] = m_BodyData.positions[from];
	m_BodyData.linearVelocities[to] = m_BodyData.linearVelocities[from];
	m_BodyData.orientations[to] = m_BodyData.orientations[from];
	m_BodyData.angularVelocities[to] = m_BodyData.angularVelocities[from];
	m_BodyData.forceAccumulators[to] = m_BodyData.forceAccumulators[from];
	m_BodyData.inverseMasses[to] = m_BodyData.inverseMasses[from];
	m_BodyData.dirtyFlags[to] = m_BodyData.dirtyFlags[from];
	m_BodyData.shapeCounts[to] = m_BodyData.shapeCounts[from];
	m_BodyData.shapeOffsets[to] = m_BodyData.shapeOffsets[from];
}

void BodyManager::PopBackDenseData() noexcept
{
	m_BodyData.positions.pop_back();
	m_BodyData.linearVelocities.pop_back();
	m_BodyData.orientations.pop_back();
	m_BodyData.angularVelocities.pop_back();
	m_BodyData.forceAccumulators.pop_back();
	m_BodyData.inverseMasses.pop_back();
	m_BodyData.dirtyFlags.pop_back();
	m_BodyData.shapeBuffer.pop_back();
	m_BodyData.shapeCounts.pop_back();
	m_BodyData.shapeOffsets.pop_back();
	m_DenseToHandle.pop_back();
}





