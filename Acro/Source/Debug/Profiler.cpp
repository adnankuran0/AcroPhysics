#include "Debug/Profiler.h"
using namespace Acro::Debug;

std::unordered_map<std::string, ProfileEntry> Profiler::m_Data;
std::chrono::high_resolution_clock::time_point Profiler::m_FrameStart;