-- premake5.lua
workspace "Acro"
   architecture "x64"
   configurations { "Debug", "Release", "Dist" }
   startproject "Sandbox"

   -- Workspace-wide build options for MSVC
   filter "system:windows"
      buildoptions { "/EHsc", "/Zc:preprocessor", "/Zc:__cplusplus" }

OutputDir = "%{cfg.system}-%{cfg.architecture}/%{cfg.buildcfg}"

include "Acro/Build-Acro.lua"
include "Tests/Build-Tests.lua"
include "Sandbox/Build-Sandbox.lua"