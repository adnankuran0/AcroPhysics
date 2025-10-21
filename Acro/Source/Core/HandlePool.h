#ifndef ACRO_HANDLE_POOL_H
#define ACRO_HANDLE_POOL_H

#include <vector>
#include <cstdint>

namespace Acro::Core {

template <typename HandleType>
class HandlePool
{
public:
	HandlePool() = default;

	uint32_t GetRefCount(const HandleType& handle) const noexcept { return m_RefCount[handle.index]; }
	inline void AddRef(const HandleType& handle) noexcept
	{
		if (!IsValid(handle)) return;

		m_RefCount[handle.index]++;
	}
	inline int ReleaseRef(const HandleType& handle) noexcept
	{
		if (!IsValid(handle)) return -1;
		if (m_RefCount[handle.index] == 0) return 0;

		return m_RefCount[handle.index]--;

	}
	inline bool IsValid(const HandleType& handle) const noexcept
	{
		if (handle.index >= m_Generations.size()) return false;
		return m_Generations[handle.index] == handle.generation && handle != HandleType::Null();

	}
	inline size_t GetDenseIndex(const HandleType& handle) const noexcept { return m_Sparse[handle.index]; }

	void CreateHandle(HandleType& outHandle, uint32_t& outDenseIndex) noexcept
	{
		if (!m_FreeHandles.empty())
		{
			outHandle = m_FreeHandles.back();
			m_FreeHandles.pop_back();
			outDenseIndex = static_cast<uint32_t>(m_DenseToHandle.size());
			m_Sparse[outHandle.index] = outDenseIndex;
			m_Generations[outHandle.index]++;
			outHandle.generation = m_Generations[outHandle.index];
			m_RefCount[outHandle.index] = 0;

		}
		else
		{
			// create new handle
			outHandle.index = static_cast<uint32_t>(m_Sparse.size());
			outHandle.generation = 0;
			m_Sparse.push_back(static_cast<uint32_t>(m_DenseToHandle.size()));
			m_RefCount.push_back(static_cast<uint32_t>(0));
			m_Generations.push_back(0);
			outDenseIndex = static_cast<uint32_t>(m_DenseToHandle.size());
		}
	}

	void DestroyHandle(const HandleType& handle, size_t lastDenseIndex, size_t denseIndex) noexcept
	{
		if (denseIndex != lastDenseIndex)
		{

			// Update sparse 
			uint32_t lastHandleIndex = m_DenseToHandle[lastDenseIndex];
			m_Sparse[lastHandleIndex] = denseIndex;
			m_DenseToHandle[denseIndex] = lastHandleIndex;
		}

		m_Generations[handle.index]++;
		m_FreeHandles.push_back(handle);
		m_RefCount[handle.index] = 0;
	}

	inline HandleType GetHandle(uint32_t denseIndex) const noexcept
	{
		HandleType handle;
		handle.index = m_DenseToHandle[denseIndex];
		handle.generation = m_Generations[denseIndex];
		return handle;
	}

protected:
	std::vector<uint32_t> m_Sparse;
	std::vector<uint32_t> m_DenseToHandle;
	std::vector<uint32_t> m_Generations;
	std::vector<HandleType> m_FreeHandles;
	std::vector<uint32_t> m_RefCount;

};

}

#endif // ACRO_HANDLE_POOL_H