#include "Collider.h"
#include <iostream>
#include <cmath>
#include "Math.h"
namespace acro {

	//define Collider
	void Collider::setPosition(const Vec2& newPosition) {}
	Vec2 Collider::getPosition() const { return Vec2(0,0); }
	float Collider::getRadius() const { return 0; }
	float Collider::getWidth() const { return 0; }
	float Collider::getHeight() const { return 0; }
	void Collider::setRotation(float angle) {}
	void Collider::updateAABB() {}	
	std::vector<Vec2> Collider::getTransformedVertices() {}




	//define CircleShape
	CircleShape::CircleShape(float centerX, float centerY, float r) : position(Vec2(centerX, centerY)), radius(r) , Collider(ShapeType::CIRCLE) 
	{
		updateAABB();
	}

	float CircleShape::getRadius() const { return radius; }

	void CircleShape::setPosition(const Vec2& newPosition) { position = newPosition; updateAABB(); }

	Vec2 CircleShape::getPosition() const { return position; }

	void CircleShape::setRotation(float angle) {}

	void CircleShape::updateAABB()
	{
		aabb.m_Left = position.x - radius;
		aabb.m_Right = position.x + radius;
		aabb.m_Top = position.y - radius;
		aabb.m_Bottom = position.y + radius;
	}

	//deleted functions
	float CircleShape::getWidth() const { return 0; }
	float CircleShape::getHeight() const { return 0; }
	std::vector<Vec2> CircleShape::getTransformedVertices() { return std::vector<Vec2>(); }


	//----------------------------------------------------------------------------------------------------


	//define RectangleShape
	RectangleShape::RectangleShape(float posX, float posY, float w, float h) : position(Vec2(posX,posY)), width(w), height(h),
		Collider(ShapeType::RECTANGLE)
	{
		vertices.reserve(4);
		vertices.push_back(Vec2(posX- w/2, posY-h/2));
		vertices.push_back(Vec2(posX + w/2, posY-h/2));
		vertices.push_back(Vec2(posX + w/2, posY + h/2));
		vertices.push_back(Vec2(posX - w/2, posY + h/2));

		transformedVertices.reserve(4);
		transformedVertices = vertices;

		updateAABB();
	}

	float RectangleShape::getHeight() const { return height; }
		
	float RectangleShape::getWidth() const { return width; }

	void RectangleShape::setPosition(const Vec2& newPosition) 
	{
		position = newPosition;
		vertices.clear();
		vertices.reserve(4);
		vertices.push_back(Vec2(position.x - width / 2, position.y - height / 2));
		vertices.push_back(Vec2(position.x + width / 2, position.y - height / 2));
		vertices.push_back(Vec2(position.x + width / 2, position.y + height / 2));
		vertices.push_back(Vec2(position.x - width / 2, position.y + height / 2));

	}

	Vec2 RectangleShape::getPosition() const { return position; }

	float RectangleShape::getRadius() const { return 0; }
	
	void RectangleShape::setRotation(float angle) {
		angle = angle * Math::PI / 180.0f;

		float s = sin(angle);
		float c = cos(angle);

		transformedVertices = vertices;


		for (int i = 0; i < vertices.size(); i++) {
			float x = transformedVertices[i].x - position.x;
			float y = transformedVertices[i].y - position.y;

			float xnew = x * c - y * s;
			float ynew = x * s + y * c;

			transformedVertices[i].x = xnew + position.x;
			transformedVertices[i].y = ynew + position.y;
		}
	}	

	
	void RectangleShape::updateAABB()
	{
		float minX = FLT_MAX;
		float minY = FLT_MAX;
		float maxX = -FLT_MAX;
		float maxY = -FLT_MAX;

		for (int i = 0; i < transformedVertices.size(); i++)
		{
			if (transformedVertices[i].x < minX)
				minX = transformedVertices[i].x;
			if (transformedVertices[i].x > maxX)
				maxX = transformedVertices[i].x;
			if (transformedVertices[i].y < minY)
				minY = transformedVertices[i].y;
			if (transformedVertices[i].y > maxY)
				maxY = transformedVertices[i].y;
		}

		aabb.m_Left = minX;
		aabb.m_Right = maxX;
		aabb.m_Top = minY;
		aabb.m_Bottom = maxY;
	}

	std::vector<Vec2> RectangleShape::getTransformedVertices() 
	{ 
		
		return transformedVertices; 
	
	}



}