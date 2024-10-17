#pragma once
#include<stdexcept>


namespace acro {
	class Math
	{
	public:
		static const float PI;
		static const float epsilon;
		static float clamp(float num, float low, float high);
		static int clamp(int num, int low, int high);
		static bool nearlyEquals(float a, float b);
		static float toRadians(float degrees);
		static float toDegrees(float radians);
	};
}