#ifndef ACRO_SHAPE_MANAGER_H
#define ACRO_SHAPE_MANAGER_H

#include <cstdint>
#include "Physics/Shape/ShapeData.h"

namespace Acro::Physics {

	struct ShapeHandle
	{
		uint32_t index;
		uint32_t generation;

		static constexpr ShapeHandle Null() { return { std::numeric_limits<uint32_t>::max(),std::numeric_limits<uint32_t>::max() }; }

		bool operator==(const ShapeHandle& other) const noexcept
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
		
		inline bool IsDirty(const ShapeHandle& handle) const noexcept { return m_ShapeData.dirtyFlags[m_Sparse[handle.index]]; }
		inline void SetDirty(const ShapeHandle& handle, uint8_t isDirty) noexcept { m_ShapeData.dirtyFlags[m_Sparse[handle.index]] = isDirty; }

		inline bool IsValid(const ShapeHandle& handle) const noexcept
		{
			if (handle.index >= m_Generations.size()) return false;
			return m_Generations[handle.index] == handle.generation;
		}

		inline Acro::ShapeType GetShapeType(const ShapeHandle& handle) const noexcept { assert(IsValid(handle)); return m_ShapeData.shapeTypes[m_Sparse[handle.index]]; }

		inline Acro::Math::Vector3 GetOffset(const ShapeHandle& handle) const noexcept { assert(IsValid(handle)); return m_ShapeData.offsets[m_Sparse[handle.index]]; }
		inline void SetOffset(const ShapeHandle& handle, const Acro::Math::Vector3& offset) noexcept
		{
			assert(IsValid(handle));
			SetDirty(handle, 1);
			m_ShapeData.offsets[m_Sparse[handle.index]] = offset;
		}

		inline Acro::Math::Vector3 GetExtent(const ShapeHandle& handle) const noexcept { assert(IsValid(handle)); return m_ShapeData.extents[m_Sparse[handle.index]]; }
		inline void SetExtent(const ShapeHandle& handle, const Acro::Math::Vector3& extent) noexcept
		{
			assert(IsValid(handle));
			SetDirty(handle, 1);
			m_ShapeData.extents[m_Sparse[handle.index]] = extent;
		}

		inline float GetRadius(const ShapeHandle& handle) const noexcept { assert(IsValid(handle)); return m_ShapeData.radii[m_Sparse[handle.index]]; }
		inline void SetRadius(const ShapeHandle& handle, float radius) noexcept 
		{ 
			assert(IsValid(handle)); 
			SetDirty(handle, 1);
			m_ShapeData.radii[m_Sparse[handle.index]] = radius; 
		}

		[[nodiscard]] Acro::Physics::ShapeData& GetData() noexcept { return m_ShapeData; }

	private:
		void DestroyShape(const ShapeHandle& handle) noexcept;
		void CreateHandle(ShapeHandle& outShapeHandle, uint32_t& outDenseIndex) noexcept;

		void SwapDenseData(size_t from, size_t to) noexcept;
		void PopBackDenseData() noexcept;

		Acro::Physics::ShapeData m_ShapeData; // dense
		std::vector<uint32_t> m_Sparse;
		std::vector<uint32_t> m_DenseToHandle;
		std::vector<uint32_t> m_RefCount;
		std::vector<uint32_t> m_Generations;
		std::vector<ShapeHandle> m_FreeHandles;
	};

}

#endif // !ACRO_SHAPE_MANAGER_H
