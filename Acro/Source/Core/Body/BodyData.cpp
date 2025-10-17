#include "BodyData.h"

using namespace Acro::Core;

void BodyData::Reserve(size_t capacity)
{
	positions.reserve(capacity);
	linearVelocities.reserve(capacity);
	orientations.reserve(capacity);
	angularVelocities.reserve(capacity);
	forceAccumulators.reserve(capacity);
	inverseMasses.reserve(capacity);
	shapeBuffer.reserve(capacity);
}