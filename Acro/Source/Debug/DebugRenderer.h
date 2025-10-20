#ifndef ACRO_DEBUG_RENDERER_H
#define ACRO_DEBUG_RENDERER_H

#include "Math/Vector3.h"
#include "Debug/DebugRendererData.h"
#include <vector>
#include <iostream>

namespace Acro::Debug {

class DebugRenderer
{
public:
	void DrawLine(const Acro::Math::Vector3& start, const Acro::Math::Vector3& end, const Acro::Math::Vector3 color,float duration = -1.0f);
	void DrawPoint(const Acro::Math::Vector3& pos, const Acro::Math::Vector3 color, float duration = -1.0f);
	void DrawTriangle(const Acro::Math::Vector3& pos1, Acro::Math::Vector3& pos2, Acro::Math::Vector3& pos3, Acro::Math::Vector3& color, float duration = -1.0f);
	void DrawAABB(Acro::Math::Vector3& min, Acro::Math::Vector3& max, Acro::Math::Vector3& color,float duration = -1.0f);
	void DrawSphere(const Acro::Math::Vector3& pos, float radius, Acro::Math::Vector3 color, float duration = -1.0f, int segments = 32);

	[[nodiscard]] const std::vector<Acro::Debug::DebugPrimitive>& GetPrimitives() const { return m_Primitives; }
	void Clear(float dt) noexcept;

	bool drawAABBs = false;
	bool drawShapes = false;
	bool drawContactPoints = false;

private:
	std::vector<Acro::Debug::DebugPrimitive> m_Primitives;

};

}

#endif // !ACRO_DEBUG_RENDERER_H
