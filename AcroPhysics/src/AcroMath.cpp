#include "AcroMath.h"


namespace acro
{
	const float Math::PI = 3.14159265358979323846f;
	const float Math::epsilon = 1e-5f;
		 
	float Math::clamp(float value, float min, float max)
	{
		if (min == max)
			return min;
		if (min > max) throw std::invalid_argument("min is greater than the max");
		if (max < min) throw std::invalid_argument("max is lesser than the min");
			
		if (value <= min)
			return min;
		else if (value >= max)
			return max;
		else
			return value;
	}

	int Math::clamp(int value, int min, int max)
	{
		if (min == max)
			return min;
		if (min > max) throw std::invalid_argument("min is greater than the max");
		if (max < min) throw std::invalid_argument("max is lesser than the min");

		if (value <= min)
			return min;
		else if (value >= max)
			return max;
		else
			return value;
	}

	float Math::toRadians(float degrees)
	{
		return degrees * PI / 180.0f;
	}

	float Math::toDegrees(float radians)
	{
		return radians * 180.0f / PI;
	}

	bool Math::nearlyEquals(float a, float b)
	{
		return fabs(a - b) < epsilon;
	}

}