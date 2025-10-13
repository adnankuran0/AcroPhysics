#ifndef ACRO_SHAPE_H
#define ACRO_SHAPE_H

#include "Core/Shape/ShapeManager.h"



namespace Acro::Core {
	struct ShapeHandle; // forward declaration
}

namespace Acro {

class Rigidbody; // forward declaration

class Shape
{
public:
	Shape(Acro::Core::ShapeManager* shapeManager, const Acro::Core::ShapeHandle& handle,Acro::Core::ShapeType shapeType) : m_ShapeManager(shapeManager), m_Handle(handle), m_ShapeType(shapeType) {}
	virtual ~Shape() = default;

	void SetOffset(const Acro::Math::Vector3& offset) { m_ShapeManager->SetOffset(m_Handle, offset); }
	Acro::Math::Vector3 GetOffset() { return m_ShapeManager->GetOffset(m_Handle); }

protected:
	friend class Rigidbody;
	friend class World;

	Acro::Core::ShapeHandle m_Handle;
	Acro::Core::ShapeManager* m_ShapeManager;
	Acro::Core::ShapeType m_ShapeType;
};

class BoxShape : public Shape
{
public:
	BoxShape(Acro::Core::ShapeManager* shapeManager, const Acro::Core::ShapeHandle& handle) : Shape(shapeManager,handle, Acro::Core::ShapeType::Box) {}

	void SetExtent(const Acro::Math::Vector3& extent) { m_ShapeManager->SetExtent(m_Handle,extent); }
	Acro::Math::Vector3 GetExtent() { return m_ShapeManager->GetExtent(m_Handle); }

};

class SphereShape : public Shape
{
public:
	SphereShape(Acro::Core::ShapeManager* shapeManager, const Acro::Core::ShapeHandle& handle) : Shape(shapeManager, handle, Acro::Core::ShapeType::Sphere) {}

	void SetRadius(float radius) { m_ShapeManager->SetRadius(m_Handle, radius); }
	float GetRadius() { return m_ShapeManager->GetRadius(m_Handle); }

};

}

#endif // !ACRO_SHAPE_H
