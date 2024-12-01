#include "Collider.h"
#include <iostream>
#include <cmath>
#include <algorithm>
#include "AcroMath.h"
namespace acro {

	void Collider::setPosition(const Vec2& newPosition) {}
	Vec2 Collider::getPosition() const { return Vec2(0, 0); }
	float Collider::getRadius() const { return 0; }
	void Collider::setRadius(float rad) {}
	float Collider::getWidth() const { return 0; }
	float Collider::getHeight() const { return 0; }
	void Collider::setRotation(float angle) {}
	float Collider::getRotation() const { return 0; }
	void Collider::setWidth(float w) {}
	void Collider::setHeight(float h) {}
	void Collider::setSize(float w, float h) {}
	Vec2 Collider::getSize() const { return Vec2(0, 0); }
	void Collider::updateAABB() {}

	// CircleShape
	CircleShape::CircleShape(float centerX, float centerY, float r)
		: position(Vec2(centerX, centerY)), radius(r), Collider(ShapeType::CIRCLE) {
		vertices.reserve(0);
		transformedVertices.reserve(0);
		updateAABB();
		area = Math::PI * r * r;
	}

	float CircleShape::getRadius() const { return radius; }
	void CircleShape::setRadius(float rad) { radius = rad; area = radius * radius * Math::PI; }

	void CircleShape::setPosition(const Vec2& newPosition) {
		if (position != newPosition) {
			position = newPosition;
			updateAABB();
		}
	}

	Vec2 CircleShape::getPosition() const { return position; }

	void CircleShape::setRotation(float angle) {}
	float CircleShape::getRotation() const { return 0; }

	void CircleShape::updateAABB() {
		aabb.m_Left = position.x - radius;
		aabb.m_Right = position.x + radius;
		aabb.m_Top = position.y - radius;
		aabb.m_Bottom = position.y + radius;
	}

	float CircleShape::getWidth() const { return 0; }
	float CircleShape::getHeight() const { return 0; }
	void CircleShape::setWidth(float w) {}
	void CircleShape::setHeight(float h) {}
	void CircleShape::setSize(float w, float h) {}
	Vec2 CircleShape::getSize() const { return Vec2(0, 0); }


	// RectangleShape
	RectangleShape::RectangleShape(float posX, float posY, float w, float h)
		: position(Vec2(posX, posY)), width(w), height(h), Collider(ShapeType::RECTANGLE) {
		vertices.reserve(4);
		transformedVertices.reserve(4);
		updateVertices();
		updateAABB();
		area = w * h;
	}

	void RectangleShape::updateVertices() {
		vertices = {
			Vec2(position.x - width / 2, position.y - height / 2),
			Vec2(position.x + width / 2, position.y - height / 2),
			Vec2(position.x + width / 2, position.y + height / 2),
			Vec2(position.x - width / 2, position.y + height / 2)
		};
		transformedVertices = vertices;
	}

	float RectangleShape::getHeight() const { return height; }
	float RectangleShape::getWidth() const { return width; }

	void RectangleShape::setPosition(const Vec2& newPosition) {
		if (position != newPosition) {
			position = newPosition;
			updateVertices();
			updateAABB();
		}
	}

	Vec2 RectangleShape::getPosition() const { return position; }
	float RectangleShape::getRadius() const { return 0; }
	void RectangleShape::setRadius(float rad) {}

	void RectangleShape::setRotation(float angle) {
		angle = Math::toRadians(angle);
		float s = sin(angle);
		float c = cos(angle);

		for (size_t i = 0; i < vertices.size(); ++i) {
			float x = vertices[i].x - position.x;
			float y = vertices[i].y - position.y;
			transformedVertices[i].x = x * c - y * s + position.x;
			transformedVertices[i].y = x * s + y * c + position.y;
		}
		updateAABB();
	}

	Vec2 RectangleShape::getSize() const { return Vec2(width, height); }

	void RectangleShape::setSize(float w, float h) {
		width = w;
		height = h;
		updateVertices();
		updateAABB();
		area = w * h;
	}

	void RectangleShape::setWidth(float w) {
		width = w;
		updateVertices();
		updateAABB();
		area = w * height;
	}

	void RectangleShape::setHeight(float h) {
		height = h;
		updateVertices();
		updateAABB();
		area = width * h;
	}

	float RectangleShape::getRotation() const { return 0; }

	

	void RectangleShape::updateAABB() {
		float minX = transformedVertices[0].x;
		float maxX = minX;
		float minY = transformedVertices[0].y;
		float maxY = minY;

		for (const auto& vertex : transformedVertices) {
			minX = std::min(minX, vertex.x);
			maxX = std::max(maxX, vertex.x);
			minY = std::min(minY, vertex.y);
			maxY = std::max(maxY, vertex.y);
		}

		aabb.m_Left = minX;
		aabb.m_Right = maxX;
		aabb.m_Top = minY;
		aabb.m_Bottom = maxY;
	}

}