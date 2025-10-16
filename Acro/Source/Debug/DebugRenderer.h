#ifndef ACRO_DEBUG_RENDERER_H
#define ACRO_DEBUG_RENDERER_H

#include "Math/Vector3.h"
#include "Debug/DebugRendererData.h"
#include <vector>

namespace Acro::Debug {

class DebugRenderer
{
public:
	void DrawLine(const Acro::Math::Vector3& start, const Acro::Math::Vector3& end, const Acro::Math::Vector3 color,float duration);
	void DrawPoint(const Acro::Math::Vector3& pos, const Acro::Math::Vector3 color, float duration);
	void DrawTriangle(const Acro::Math::Vector3& pos1, Acro::Math::Vector3& pos2, Acro::Math::Vector3& pos3, Acro::Math::Vector3& color, float duration);
	void DrawAABB(Acro::Math::Vector3& min, Acro::Math::Vector3& max, Acro::Math::Vector3& color,float duration);
	void DrawSphere(const Acro::Math::Vector3& pos, float radius, Acro::Math::Vector3 color, float duration, int segments);

	[[nodiscard]] const std::vector<Acro::Debug::DebugPrimitive>& GetPrimitives() const { return m_Primitives; }
	void Clear() { m_Primitives.clear(); }

	bool drawAABBs = false;
	bool drawShapes = false;

private:
	std::vector<Acro::Debug::DebugPrimitive> m_Primitives;

};

}

#endif // !ACRO_DEBUG_RENDERER_H
