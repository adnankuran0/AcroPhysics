#ifndef ACRO_CONTACT_MANIFOLD_H
#define ACRO_CONTACT_MANIFOLD_H

#include "Physics/Shape/ShapeInstanceManager.h"
#include "Physics/Collision/ContactPoint.h"

namespace Acro::Physics {

#define MAX_CONTACT_POINTS 4

struct ContactManifold
{
	Acro::Physics::ShapeInstanceHandle a, b;
	ContactPoint points[MAX_CONTACT_POINTS];
	int count;
};

}


#endif // !ACRO_CONTACT_MANIFOLD_H
