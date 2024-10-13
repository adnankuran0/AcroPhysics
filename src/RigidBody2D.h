#pragma once
#include "Vec2.h"
#include <iostream>
#include "CollisionShape.h"
#include <memory>

namespace acro {

	class RigidBody2D
	{
	public:
		Vec2 position;
		Vec2 velocity;
		Vec2 force;
		float rotation;
		float rotationalVelocity;
		float mass;
		bool isStatic;
		float restitution;
		float friction = 0.995f;
		bool isInCollision = false;
		CollisionShape* collisionShape;
		ShapeType GetShapeType() const; 		
		RigidBody2D(Vec2 pos, float m,bool isStatic = false, float restitution = 0.5);
		RigidBody2D(const RigidBody2D& other);
		~RigidBody2D();
		void applyForce(const Vec2& f);
		void applyForce(float x, float y);
		void setCollisionShape(float radius);
		void setCollisionShape(float width,float height);
		void move(const Vec2& displacement);
		void moveTo(const Vec2& position);
		

	};
}
