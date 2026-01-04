#include "Rigidbody.h"
#include "World.h"

Acro::ShapeInstance Acro::Rigidbody::AttachShape(const Acro::Shape& shape)
{
	m_ShapeHandle = shape.m_Handle;
	return m_World->AttachShape(*this, shape);
}

void Acro::Rigidbody::DetachShape(const Acro::Shape& shape)
{
	m_World->DetachShape(*this, shape);
}