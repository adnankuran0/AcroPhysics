#ifndef ACRO_INTEGRATOR_H
#define ACRO_INTEGRATOR_H

#include "Core/BodyManager.h"

namespace Acro {

class Integrator
{
public:
	void Step(BodyData& bodyData, Vector3& gravity, float deltaTime);
private:
	void IntegrateBody(BodyData& bodyData, Vector3& gravity,size_t index, float deltaTime);
};

}

#endif // !ACRO_INTEGRATOR_H

