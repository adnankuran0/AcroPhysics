#include "BodyManager.h"

using namespace Acro::Core;

BodyHandle BodyManager::CreateBody() noexcept
{
	BodyHandle handle;
	uint32_t denseIndex;

	if (!m_FreeHandles.empty())
	{
		handle = m_FreeHandles.back();
		m_FreeHandles.pop_back();
		denseIndex = m_BodyData.positions.size();
		m_Sparse[handle.index] = denseIndex;
		m_Generations[handle.index]++;
	}
	else
	{
		// create new handle
		handle.index = static_cast<uint32_t>(m_Sparse.size());
		handle.generation = 0;
		m_Sparse.push_back(static_cast<uint32_t>(m_BodyData.positions.size()));
		m_Generations.push_back(0);
		denseIndex = m_BodyData.positions.size();
	}

	m_BodyData.positions.push_back(Vector3());
	m_BodyData.linearVelocities.push_back(Vector3());
	m_BodyData.orientations.push_back(Quaternion());
	m_BodyData.angularVelocities.push_back(Vector3());
	m_BodyData.forceAccumulators.push_back(Vector3());
	m_BodyData.masses.push_back(1.0f);

	m_DenseToHandle.push_back(handle.index);
	return { handle.index,m_Generations[handle.index] };

}

void BodyManager::DestroyBody(const BodyHandle& handle) noexcept
{
	if (!IsValid(handle)) return;

	uint32_t denseIndex = m_Sparse[handle.index];
	uint32_t lastDense = m_BodyData.positions.size() - 1;

	if (denseIndex != lastDense)
	{
		// Swap dense data
		m_BodyData.positions[denseIndex] = m_BodyData.positions[lastDense];
		m_BodyData.linearVelocities[denseIndex] = m_BodyData.linearVelocities[lastDense];
		m_BodyData.orientations[denseIndex] = m_BodyData.orientations[lastDense];
		m_BodyData.angularVelocities[denseIndex] = m_BodyData.angularVelocities[lastDense];
		m_BodyData.forceAccumulators[denseIndex] = m_BodyData.forceAccumulators[lastDense];
		m_BodyData.masses[denseIndex] = m_BodyData.masses[lastDense];

		// Update sparse 
		uint32_t lastHandleIndex = m_DenseToHandle[lastDense];
		m_Sparse[lastHandleIndex] = denseIndex;
		m_DenseToHandle[denseIndex] = lastHandleIndex;
	}

	// Pop back dense data
	m_BodyData.positions.pop_back();
	m_BodyData.linearVelocities.pop_back();
	m_BodyData.orientations.pop_back();
	m_BodyData.angularVelocities.pop_back();
	m_BodyData.forceAccumulators.pop_back();
	m_BodyData.masses.pop_back();
	m_DenseToHandle.pop_back();

	m_Generations[handle.index]++;
	m_FreeHandles.push_back(handle);
}
