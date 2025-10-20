#ifndef ACRO_NARROWPHASE_H
#define ACRO_NARROWPHASE_H

#include <vector>
#include "Physics/Collision/ContactManifold.h"
#include "Physics/Shape/ShapeManager.h"
#include "Physics/Collision/CollisionPair.h"

namespace Acro::Physics {

class Narrowphase
{
public:
	std::vector<Acro::Physics::ContactManifold> Compute(Acro::Physics::ShapeManager& sm, 
		Acro::Physics::ShapeInstanceManager& sim ,
		const std::vector<std::pair<Acro::Physics::ShapeInstanceHandle, Acro::Physics::ShapeInstanceHandle>>& pairs);

private:
	bool SphereSphere(
		const Acro::Physics::CollisionPair& pair,
		Acro::Physics::ShapeManager& sm,
		Acro::Physics::ShapeInstanceManager& sim,
		Acro::Physics::ContactManifold& manifold);

	bool SphereBox(
		const Acro::Physics::CollisionPair& pair,
		Acro::Physics::ShapeManager& sm,
		Acro::Physics::ShapeInstanceManager& sim,
		Acro::Physics::ContactManifold& manifold);

	bool BoxBox(
		const Acro::Physics::CollisionPair& pair,
		Acro::Physics::ShapeManager& sm,
		Acro::Physics::ShapeInstanceManager& sim,
		Acro::Physics::ContactManifold& manifold);
};

}

#endif // !ACRO_NARROWPHASE_H
