#include "DebugRenderer.h"

using namespace Acro::Debug;
using namespace Acro::Math;


void DebugRenderer::DrawLine(const Acro::Math::Vector3& start, const Acro::Math::Vector3& end, const Acro::Math::Vector3 color, float duration)
{
	DebugPrimitive line;
	line.type = DebugPrimitiveType::Line;
	line.vertices.push_back({ start,color });
	line.vertices.push_back({ end,color });
	line.duration = duration;
	m_Primitives.push_back(line);
}

void DebugRenderer::DrawPoint(const Acro::Math::Vector3& pos, const Acro::Math::Vector3 color, float duration)
{
	DebugPrimitive point;
	point.type = DebugPrimitiveType::Point;
	point.vertices.push_back({ pos,color });
	point.duration = duration;
	m_Primitives.push_back(point);
}

void DebugRenderer::DrawTriangle(const Acro::Math::Vector3& pos1, Acro::Math::Vector3& pos2, Acro::Math::Vector3& pos3, Acro::Math::Vector3& color, float duration)
{
	DrawLine(pos1, pos2, color, duration);
	DrawLine(pos2, pos3, color, duration);
	DrawLine(pos1, pos3, color, duration);
}

void DebugRenderer::DrawAABB(Acro::Math::Vector3& min, Acro::Math::Vector3& max, Acro::Math::Vector3& color, float duration)
{
	Vector3 v[8] = { {min.x,min.y,min.z},{max.x,min.y,min.z}, {max.x,max.y,min.z}, {min.x,max.y,min.z},
					 {min.x,min.y,max.z},{max.x,min.y,max.z}, {max.x,max.y,max.z}, {min.x,max.y,max.z} };

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


