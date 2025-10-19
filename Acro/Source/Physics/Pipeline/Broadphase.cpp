#include "Broadphase.h"

#include <algorithm>

using namespace Acro::Physics;

using Pair = std::pair<ShapeInstanceHandle, ShapeInstanceHandle>;


std::vector<Pair> Broadphase::Compute(ShapeInstanceManager* shapeInsanceManager)
{
	std::vector<Pair> out;

	auto& data = shapeInsanceManager->GetData();

	size_t n = data.worldAABBs.size();
	if (n == 0) return out;

	std::vector<Endpoint> endPoints;
	endPoints.reserve(n * 2);

	for (uint32_t i = 0; i < n; i++)
	{
		const auto& aabb = data.worldAABBs[i];
		endPoints.push_back({ aabb.min.x,i,true });
		endPoints.push_back({ aabb.max.x,i,false });
	}

	std::sort(endPoints.begin(), endPoints.end(), [](const Endpoint& a, const Endpoint& b)
		{
			if (a.value == b.value) return a.isMin && !b.isMin; // min first
			return a.value < b.value;
		});

	std::vector<uint32_t> active;
	active.reserve(n);

	for (const auto& endpoint : endPoints)
	{
		if (endpoint.isMin)
		{
			// test with actives
			for (uint32_t j : active)
			{
				// check collision filter
				const auto& filterA = data.filters[endpoint.index];
				const auto& filterB = data.filters[j];
				if (!CollisonLayer::ShouldCollide(filterA, filterB)) continue;

				// check y and z
				const auto& A = data.worldAABBs[endpoint.index];
				const auto& B = data.worldAABBs[j];

				bool overlapY = !(A.max.y < B.min.y || B.max.y < A.min.y);
				bool overlapZ = !(A.max.z < B.min.z || B.max.z < A.min.z);
				if (overlapY && overlapZ)
				{
					
					ShapeInstanceHandle ha = shapeInsanceManager->GetHandle(endpoint.index);
					ShapeInstanceHandle hb = shapeInsanceManager->GetHandle(j);
					out.emplace_back(ha,hb);
				}
			}

			active.push_back(endpoint.index);
		}
		else
		{
			// deactivate
			auto it = std::find(active.begin(), active.end(), endpoint.index);
			if (it != active.end()) active.erase(it);
		}
	}

	return out;

}
