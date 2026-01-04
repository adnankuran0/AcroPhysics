#pragma once

#include <filesystem>


class Path
{
public:
	inline static void Init(const char* exePath)
	{
		m_ProjectRoot = std::filesystem::path(exePath)
			.parent_path()
			.parent_path()
			.parent_path()
			.parent_path()
			.parent_path() 
			/ "Sandbox" 
			/ "Source";
	}

	inline static std::filesystem::path GetProjectRoot() noexcept { return m_ProjectRoot; }
	inline static std::filesystem::path GetShadersDir() noexcept { return m_ProjectRoot / "Shaders"; }
	inline static std::filesystem::path GetTexturesDir() noexcept { return m_ProjectRoot / "Textures"; }
private:
	inline static std::filesystem::path m_ProjectRoot;
};
