#ifndef ACRO_COLLISION_PAIR_H
#define ACRO_COLLISION_PAIR_H

#include "Physics/Shape/ShapeInstanceManager.h"

namespace Acro::Physics {

struct CollisionPair
{
	Acro::Physics::ShapeHandle shapeA;
	Acro::Physics::ShapeHandle shapeB;
	Acro::Physics::ShapeInstanceHandle shapeInstanceA;
	Acro::Physics::ShapeInstanceHandle shapeInstanceB;
};

}

#endif // !ACRO_COLLISION_PAIR_H
