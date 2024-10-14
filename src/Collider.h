#pragma once
#include "Vec2.h"
#include "Math.h"
#include <algorithm>
#include <vector>
#include "AABB.h"

namespace acro {


	enum class ShapeType {
		CIRCLE = 0,
		RECTANGLE = 1
	};


	class Collider
	{
	public:
		Collider(ShapeType t) : type(t) {}
		~Collider() {}

		std::vector<Vec2> vertices;
		std::vector<Vec2> transformedVertices;

		AABB aabb;
		ShapeType type;

		bool isTransformUpdateRequired = true;

		virtual void setPosition(const Vec2& newPosition);
		virtual Vec2 getPosition() const;
		virtual void setRotation(float angle);
		virtual float getRadius() const;
		virtual float getWidth() const;
		virtual float getHeight() const;
		virtual void updateAABB();
		virtual std::vector<Vec2> getTransformedVertices();
	
	};



	class CircleShape : public Collider
	{
	public:
		CircleShape(float centerX, float centerY, float r);

		void setPosition(const Vec2& newPosition) override;
		Vec2 getPosition() const override;
		void setRotation(float angle) override;
		float getRadius() const override;
		void updateAABB() override;	

		std::vector<Vec2> getTransformedVertices() override;

		//deleted functions
		float getWidth() const override;
		float getHeight() const override;
	private:
		Vec2 position = Vec2(0, 0);
		float radius;
	};



	class RectangleShape : public Collider
	{
	public:
		RectangleShape(float posX, float posY, float w, float h);

		void setPosition(const Vec2& newPosition) override;
		Vec2 getPosition() const override;
		void setRotation(float angle) override;
		float getWidth() const override;
		float getHeight() const override;
		void updateAABB() override;

		std::vector<Vec2> getTransformedVertices() override;
		
		//deleted functions
		float getRadius() const override;
	private:
		float width, height;
		Vec2 position = Vec2(0, 0);
	};
}


