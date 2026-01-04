#pragma once

class Timer
{
public:
	inline static void Update(double glfwTime) noexcept 
	{
		float currentFrame = static_cast<float>(glfwTime);
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;
	}
	inline static float deltaTime = 0.0f;
private:
	inline static float lastFrame = 0.0f;
};