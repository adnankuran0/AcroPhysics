-- premake5.lua
workspace "Acro"
   architecture "x64"
   configurations { "Debug", "Release", "Dist" }
   startproject "Sandbox"

   -- Workspace-wide build options for MSVC
   filter "system:windows"
      buildoptions { "/EHsc", "/Zc:preprocessor", "/Zc:__cplusplus" }

OutputDir = "%{cfg.system}-%{cfg.architecture}/%{cfg.buildcfg}"

group "Acro"
	include "Acro/Build-Acro.lua"
group "Tests"
	include "Tests/Build-Tests.lua"
group ""

include "Sandbox/Build-Sandbox.lua"