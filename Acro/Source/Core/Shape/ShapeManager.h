#ifndef ACRO_SHAPE_MANAGER_H
#define ACRO_SHAPE_MANAGER_H

#include <cstdint>
#include "Core/Shape/ShapeData.h"

namespace Acro::Core {

	struct ShapeHandle
	{
		uint32_t index;
		uint32_t generation;

		bool operator==(const ShapeHandle& other) const
		{
			return index == other.index && generation == other.generation;
		}

	};

	class ShapeManager
	{
	public:
		ShapeHandle CreateBoxShape(const Acro::Math::Vector3& extent = Acro::Math::Vector3(1.0), const Acro::Math::Vector3& offset = Acro::Math::Vector3(0.0f) ) noexcept;
		ShapeHandle CreateSphereShape(float radius = 1.0f, const Acro::Math::Vector3& offset = Acro::Math::Vector3(0.0f)) noexcept;

		void AddRef(const ShapeHandle& handle);
		uint32_t GetRefCount(const ShapeHandle& handle) { return m_RefCount[handle.index]; }
		void ReleaseRef(const ShapeHandle& handle);
		

		inline bool IsValid(const ShapeHandle& handle) const noexcept
		{
			if (handle.index >= m_Generations.size()) return false;
			return m_Generations[handle.index] == handle.generation;
		}

		inline Acro::Math::Vector3 GetOffset(const ShapeHandle& handle) const noexcept { assert(IsValid(handle)); return m_ShapeData.offsets[m_Sparse[handle.index]]; }
		inline Acro::Math::Vector3 GetExtent(const ShapeHandle& handle) const noexcept { assert(IsValid(handle)); return m_ShapeData.extents[m_Sparse[handle.index]]; }
		inline float GetRadius(const ShapeHandle& handle) const noexcept { assert(IsValid(handle)); return m_ShapeData.radii[m_Sparse[handle.index]]; }

		inline void SetOffset(const ShapeHandle& handle, const Acro::Math::Vector3& offset) noexcept { assert(IsValid(handle));  m_ShapeData.offsets[m_Sparse[handle.index]] = offset; }
		inline void SetExtent(const ShapeHandle& handle, const Acro::Math::Vector3& extent) noexcept { assert(IsValid(handle)); m_ShapeData.extents[m_Sparse[handle.index]] = extent; }
		inline void SetRadius(const ShapeHandle& handle, float radius) noexcept { assert(IsValid(handle)); m_ShapeData.radii[m_Sparse[handle.index]] = radius; }
		
		[[nodiscard]] ShapeData& GetData() noexcept { return m_ShapeData; }

	private:
		void DestroyShape(const ShapeHandle& handle) noexcept;

		ShapeData m_ShapeData; // dense
		std::vector<uint32_t> m_Sparse;
		std::vector<uint32_t> m_DenseToHandle;
		std::vector<uint32_t> m_RefCount;
		std::vector<uint32_t> m_Generations;
		std::vector<ShapeHandle> m_FreeHandles;
	};

}

#endif // !ACRO_SHAPE_MANAGER_H
