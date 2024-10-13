#include "Collider.h"
#include <iostream>
#include <cmath>
#include "Math.h"
namespace acro {


	void Collider::setPosition(const Vec2& newPosition) {}
	Vec2 Collider::getPosition() const { return Vec2(0,0); }
	float Collider::getRadius() const { return 0; }
	float Collider::getWidth() const { return 0; }
	float Collider::getHeight() const { return 0; }
	void Collider::setRotation(float angle) {}

	CircleShape::CircleShape(float centerX, float centerY, float r) : position(Vec2(centerX, centerY)), radius(r) , Collider(ShapeType::CIRCLE) {}

	float CircleShape::getRadius() const { return radius; }

	void CircleShape::setPosition(const Vec2& newPosition) { position = newPosition; }

	Vec2 CircleShape::getPosition() const { return position; }

	float CircleShape::getHeight() const { return 0; }

	float CircleShape::getWidth() const { return 0; }

	void CircleShape::setRotation(float angle) {}







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



	


}