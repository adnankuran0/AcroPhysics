#include "DebugRenderer.h"
#include "Math/Constants.h"

using namespace Acro::Debug;
using namespace Acro::Math;

void DebugRenderer::DrawLine(const Vector3& start, const Vector3& end, const Vector3 color, float duration)
{
	DebugPrimitive line;
	line.type = DebugPrimitiveType::Line;
	line.vertices.push_back({ start,color });
	line.vertices.push_back({ end,color });
	line.duration = duration;
	m_Primitives.push_back(line);
}

void DebugRenderer::DrawPoint(const Vector3& pos, const Vector3 color, float duration)
{
	DebugPrimitive point;
	point.type = DebugPrimitiveType::Point;
	point.vertices.push_back({ pos,color });
	point.duration = duration;
	m_Primitives.push_back(point);
}

void DebugRenderer::DrawTriangle(const Vector3& pos1, Vector3& pos2, Vector3& pos3, Vector3& color, float duration)
{
	m_Primitives.reserve(m_Primitives.size() + 3);
	DrawLine(pos1, pos2, color, duration);
	DrawLine(pos2, pos3, color, duration);
	DrawLine(pos1, pos3, color, duration);
}

void DebugRenderer::DrawAABB(Vector3& min, Vector3& max, Vector3& color, float duration)
{
	Vector3 v[8] = { {min.x,min.y,min.z},{max.x,min.y,min.z}, {max.x,max.y,min.z}, {min.x,max.y,min.z},
					 {min.x,min.y,max.z},{max.x,min.y,max.z}, {max.x,max.y,max.z}, {min.x,max.y,max.z} };

	m_Primitives.reserve(m_Primitives.size() + 12);

	DrawLine(v[0], v[1], color, duration);
	DrawLine(v[1], v[2], color, duration);
	DrawLine(v[2], v[3], color, duration);
	DrawLine(v[3], v[0], color, duration);
	DrawLine(v[4], v[5], color, duration);
	DrawLine(v[5], v[6], color, duration);
	DrawLine(v[6], v[7], color, duration);
	DrawLine(v[7], v[4], color, duration);
	DrawLine(v[0], v[4], color, duration);
	DrawLine(v[1], v[5], color, duration);
	DrawLine(v[2], v[6], color, duration);
	DrawLine(v[3], v[7], color, duration);
}

void DebugRenderer::DrawSphere(const Vector3& pos, float radius, Vector3 color, float duration, int segments)
{
	m_Primitives.reserve(m_Primitives.size() + segments * 3);

	for (int i = 0; i < segments; i++)
	{
		float theta = i * 2.0f * PI / segments;
		float nextTheta = ((i + 1) % segments) * 2.0f * PI / segments;

		// XZ plane
		Vector3 p0(pos.x + radius * cos(theta), pos.y, pos.z + radius * sin(theta));
		Vector3 p1(pos.x + radius * cos(nextTheta), pos.y, pos.z + radius * sin(nextTheta));
		DrawLine(p0, p1, color, duration);

		// YZ plane
		Vector3 p2(pos.x, pos.y + radius * cos(theta), pos.z + radius * sin(theta));
		Vector3 p3(pos.x, pos.y + radius * cos(nextTheta), pos.z + radius * sin(nextTheta));
		DrawLine(p2, p3, color, duration);

		// XY plane 
		Vector3 p4(pos.x + radius * cos(theta), pos.y + radius * sin(theta), pos.z);
		Vector3 p5(pos.x + radius * cos(nextTheta), pos.y + radius * sin(nextTheta), pos.z);
		DrawLine(p4, p5, color, duration);

	}
}


void DebugRenderer::Clear(float dt) noexcept
{
	size_t i = 0;
	while (i < m_Primitives.size())
	{
		m_Primitives[i].duration -= dt;

		if (m_Primitives[i].duration <= 0.0f)
		{
			m_Primitives[i] = m_Primitives.back();
			m_Primitives.pop_back();
		}
		else
		{
			++i;
		}
	}
}