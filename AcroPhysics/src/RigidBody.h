#pragma once
#include "Vec2.h"
#include <iostream>
#include "Collider.h"
#include <memory>

namespace acro {

	class RigidBody
	{
	public:

		ShapeType GetShapeType() const; 		
		RigidBody(Vec2 pos, float m,bool isStatic = false, float restitution = 0.5);
		RigidBody(const RigidBody& other);
		~RigidBody();
		void applyForce(const Vec2& f);
		void applyForce(float x, float y);
		void move(const Vec2& displacement);
		void moveTo(const Vec2& position);
		void step(float deltaTime,int iterations, const Vec2& gravity,float timeScale = 1.0f);

		void setPosition(const Vec2& pos);
		void setPosition(float x, float y);
		Vec2 getPosition() const;
		void setVelocity(const Vec2& vel);
		void setVelocity(float x, float y);
		Vec2 getVelocity() const;
		void setForce(const Vec2& f);
		void setForce(float x, float y);
		Vec2 getForce() const;
		void setRotation(float r);
		float getRotation() const;
		void setRotationalVelocity(float v);
		float getRotationalVelocity() const;
		void setCollider(float radius);
		void setCollider(float width,float height);
		Collider* getCollider();
		void setMass(float m);
		float getMass() const;
		void setInverseMass(float invM);
		float getInverseMass() const;
		void setInertia(float i);
		float getInertia() const;
		float getInverseInertia() const;
		void setStaticMode(bool isStatic);
		void applyGravity(bool useGravity);
		void setRestitution(float r);
		float getRestitution() const;
		bool getIsStatic() const;

	private:
		Vec2 position;
		Vec2 velocity;
		Vec2 force;
		float rotation;
		float rotationalVelocity;
		float mass;
		float inverseMass;
		float inertia;
		float inverseInertia;
		bool isStatic;
		bool useGravity = true;
		float restitution;
		Collider* collider;
		float calculateMomentOfInertia();
		void updateInverseMass();
		void updateInertia();
		void updateColliderPosition();
		void updateColliderRotation();
		

	};
}
