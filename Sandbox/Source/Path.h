#pragma once

#include <filesystem>


class Path
{
public:
	inline static std::filesystem::path GetProjectRoot() noexcept { return m_ProjectRoot; }
	inline static std::filesystem::path GetShadersDir() noexcept { return m_ProjectRoot / "Shaders"; }
	inline static std::filesystem::path GetTexturesDir() noexcept { return m_ProjectRoot / "Textures"; }
private:
	inline static std::filesystem::path m_ProjectRoot = std::filesystem::current_path() / "Source";
};
