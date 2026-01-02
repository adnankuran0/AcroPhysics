#ifndef ACRO_SOLVER_H

#include <vector>
#include "Physics/Collision/ContactManifold.h"

namespace Acro::Physics {

class Solver
{
public:
	void Solve(
		const std::vector<Acro::Physics::ContactManifold>& manifolds,
		Acro::Physics::ShapeInstanceManager& sim,
		Acro::Physics::BodyManager& bm,
		float deltaTime) noexcept;

	inline void SetIterations(unsigned int iterations) noexcept
	{
		assert(iterations != 0);
		m_Iterations = iterations;
	}

private:
	unsigned int m_Iterations = 10;
};

}

#endif // !ACRO_SOLVER_H
