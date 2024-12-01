#pragma once

struct PhysicsMaterial
{
	float density;
	float restitution;
	float staticFriction;
	float dynamicFriction;
	PhysicsMaterial(float density, float restitution, float staticFriction, float dynamicFriction) : density(density), restitution(restitution),
		staticFriction(staticFriction), dynamicFriction(dynamicFriction) {}
	PhysicsMaterial() : density(1.0f), restitution(0.5f), staticFriction(0.5f), dynamicFriction(0.3f) {}
};

