#pragma once
#include "Vec2.h"
#include <iostream>
#include "Collider.h"
#include <memory>

namespace acro {

	class RigidBody
	{
	public:
		Vec2 position;
		Vec2 velocity;
		Vec2 force;
		float rotation;
		float rotationalVelocity;
		float mass;
		float inverseMass;
		bool isStatic;
		bool useGravity = true;
		float restitution;
		Collider* collider;
		ShapeType GetShapeType() const; 		
		RigidBody(Vec2 pos, float m,bool isStatic = false, float restitution = 0.5);
		RigidBody(const RigidBody& other);
		~RigidBody();
		void applyForce(const Vec2& f);
		void applyForce(float x, float y);
		void setCollider(float radius);
		void setCollider(float width,float height);
		void move(const Vec2& displacement);
		void moveTo(const Vec2& position);
		void step(float deltaTime,int iterations, const Vec2& gravity,float timeScale = 1.0f);
		

	};
}
