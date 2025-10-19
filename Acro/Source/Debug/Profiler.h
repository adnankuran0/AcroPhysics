#ifndef ACRO_PROFILER_H
#define ACRO_PROFILER_H

#include <unordered_map>
#include <string>
#include <chrono>
#include <iostream>

namespace Acro::Debug {


struct ProfileEntry
{
	double totalTime = 0.0;
	size_t callCount = 0;
};

class Profiler
{
public:
	static void BeginFrame() 
	{
		m_Data.clear();
		m_FrameStart = Clock::now();
	}

	static void EndFrame()
	{
		auto frameEnd = Clock::now();
		double frameTime = std::chrono::duration<double, std::milli>(frameEnd - m_FrameStart).count();
		m_Data["FrameTotal"].totalTime = frameTime;
		m_Data["FrameTotal"].callCount = 1;
	}

	static void AddSample(const std::string& name, double ms)
	{

		auto& entry = m_Data[name];
		entry.totalTime += ms;
		entry.callCount++;
	}

	static void PrintSummary()
	{
		std::cout << "----- Frame Profile -----\n";
		for (const auto& [name, entry] : m_Data)
		{
			std::cout << name << ": " << entry.totalTime << "ms (Call count: " << entry.callCount << ")\n";
		}
		std::cout << "-------------------------\n";

	}

private:
	using Clock = std::chrono::high_resolution_clock;
	static std::unordered_map<std::string, ProfileEntry> m_Data;
	static Clock::time_point m_FrameStart;
};


class ProfilerScope
{
public:
	ProfilerScope(const char* name) : m_Name(name), m_Start(Clock::now()) {}
	~ProfilerScope()
	{
		auto end = Clock::now();
		double ms = std::chrono::duration<double, std::milli>(end - m_Start).count();
		Profiler::AddSample(m_Name, ms);
	}


private:
	using Clock = std::chrono::high_resolution_clock;
	Clock::time_point m_Start;
	const char* m_Name;
};

}

#endif // !ACRO_PROFILER_H
