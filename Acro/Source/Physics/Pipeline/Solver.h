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
	float ComputeNormalImpulse(
		float velocityNormal,
		float invMassSum,
		float restitution) noexcept;
	void ApplyImpulse(
		const Acro::Physics::BodyHandle& a,
		const Acro::Physics::BodyHandle& b, 
		float invMassA,
		float invMassB,
		const Acro::Math::Vector3& impulse,
		Acro::Physics::BodyManager& bm) noexcept;
	void SolveFriction(
		const Acro::Math::Vector3& rv,
		const Acro::Math::Vector3& normal,
		float normalImpulse,
		float invMassSum,
		const Acro::Physics::BodyHandle& a,
		const Acro::Physics::BodyHandle& b,
		Acro::Physics::BodyManager& bm) noexcept;
	void SolveContactPoint(
		const Acro::Physics::ContactManifold& manifold,
		const Acro::Physics::ContactPoint& point,
		Acro::Physics::ShapeInstanceManager& sim,
		Acro::Physics::BodyManager& bm) noexcept;
	void SolveVelocityConstraints(
		const std::vector<Acro::Physics::ContactManifold>& manifolds,
		Acro::Physics::ShapeInstanceManager& sim,
		Acro::Physics::BodyManager& bm) noexcept;
	void SolvePositionConstraints(
		const std::vector<Acro::Physics::ContactManifold>& manifolds, 
		Acro::Physics::ShapeInstanceManager& sim,
		Acro::Physics::BodyManager& bm) noexcept;


	unsigned int m_Iterations = 10;
};

}

#endif // !ACRO_SOLVER_H
