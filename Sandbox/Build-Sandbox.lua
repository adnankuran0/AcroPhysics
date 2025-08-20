project "Sandbox"
   kind "ConsoleApp"
   language "C++"
   cppdialect "C++20"
   targetdir "Binaries/%{cfg.buildcfg}"
   staticruntime "off"

   files { "Source/**.h", "Source/**.cpp", "../Vendor/GLAD/src/**.c" }

   includedirs
   {
      "Source",

	  -- Include Core
	  "../Acro/Source",
      "../Vendor/GLAD/include",
      "../Vendor/GLFW/include",
      "../Vendor/glm/include"
   }

   libdirs { "../Vendor/GLFW/lib" }

   links
   {
      "Acro",
      "glfw3"
   }

   targetdir ("../Binaries/" .. OutputDir .. "/%{prj.name}")
   objdir ("../Binaries/Intermediates/" .. OutputDir .. "/%{prj.name}")

   filter "system:windows"
       systemversion "latest"
       defines { "WINDOWS", "GLFW_STATIC" }

   filter "configurations:Debug"
       defines { "DEBUG" }
       runtime "Debug"
       symbols "On"

   filter "configurations:Release"
       defines { "RELEASE" }
       runtime "Release"
       optimize "On"
       symbols "On"

   filter "configurations:Dist"
       defines { "DIST" }
       runtime "Release"
       optimize "On"
       symbols "Off"