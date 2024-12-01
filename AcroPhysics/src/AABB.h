#pragma once

class AABB
{
public:
	float m_Left, m_Right, m_Top, m_Bottom;
	AABB(float left, float right, float top, float bottom);
	AABB() = default;
};