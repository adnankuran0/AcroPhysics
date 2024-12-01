#pragma once
#include "Vec2.h"
#include "AcroMath.h"
#include <algorithm>
#include <vector>
#include "AABB.h"

namespace acro {


	enum class ShapeType {
		CIRCLE = 0,
		RECTANGLE = 1,
		NONE = 2
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
		float area;


		virtual void setPosition(const Vec2& newPosition);
		virtual Vec2 getPosition() const;
		virtual void setRotation(float angle);
		virtual float getRotation() const; //
		virtual float getRadius() const; //
		virtual void setRadius(float rad);
		virtual float getWidth() const;
		virtual void setWidth(float w);
		virtual float getHeight() const; //
		virtual void setHeight(float h); //
		virtual void setSize(float w,float h); //
		virtual Vec2 getSize() const;
		virtual void updateAABB();

	};



	class CircleShape : public Collider
	{
	public:
		CircleShape(float centerX, float centerY, float r);

		void setPosition(const Vec2& newPosition) override;
		Vec2 getPosition() const override;
		void setRotation(float angle) override;
		float getRotation() const override;
		float getRadius() const override;
		void setRadius(float rad) override;
		void updateAABB() override;	


		//deleted functions
		float getWidth() const override;
		void setWidth(float w) override;
		float getHeight() const override;
		void setHeight(float h) override ; 
		void setSize(float w, float h) override;
		Vec2 getSize() const override;
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
		float getRotation() const override;
		float getWidth() const override;
		void setWidth(float w) override;
		float getHeight() const override;
		void setHeight(float h) override;
		void updateAABB() override;
		void updateVertices();	
		Vec2 getSize() const override;
		void setSize(float w, float h) override;

		
		//deleted functions
		float getRadius() const override;
		void setRadius(float rad) override;

	private:
		float width, height;
		Vec2 position = Vec2(0, 0);
	};
}


