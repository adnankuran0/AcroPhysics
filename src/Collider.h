#pragma once
#include "Vec2.h"
#include "Math.h"
#include <algorithm>
#include <vector>

namespace acro {


	enum class ShapeType {
		CIRCLE = 0,
		RECTANGLE = 1
	};


	class Collider
	{
	public:
		std::vector<Vec2> vertices;
		std::vector<Vec2> transformedVertices;

		ShapeType type;
		virtual void setPosition(const Vec2& newPosition);
		virtual Vec2 getPosition() const;
		virtual float getRadius() const;
		virtual float getWidth() const;
		virtual float getHeight() const;
		virtual void setRotation(float angle);
	protected:
		Collider(ShapeType t) : type(t) {}
	};



	class CircleShape : public Collider
	{
	public:

		CircleShape(float centerX, float centerY, float r);

		float getRadius() const override;
		float getWidth() const override;
		float getHeight() const override;
		Vec2 getPosition() const override;
		void setPosition(const Vec2& newPosition) override;
		void setRotation(float angle) override;
	private:
		Vec2 position = Vec2(0, 0);
		float radius;
	};



	class RectangleShape : public Collider
	{
	public:
		
		RectangleShape(float posX, float posY, float w, float h);
		Vec2 getPosition() const override;
		float getWidth() const override;
		float getHeight() const override;
		float getRadius() const override;
		void setPosition(const Vec2& newPosition) override;
		void setRotation(float angle) override;
		
	private:
		float width, height;
		Vec2 position = Vec2(0, 0);
	};
}


