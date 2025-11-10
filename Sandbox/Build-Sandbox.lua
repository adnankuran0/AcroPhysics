project "Sandbox"
   kind "ConsoleApp"
  architecture "x86_64"
  buildoptions { "-march=x86-64", "-mtune=generic" }

   language "C++"
   cppdialect "C++20"
   staticruntime "off"

   files {
      "Source/**.h",
      "Source/**.cpp",
      "../Vendor/GLAD/src/**.c",
      "../Vendor/ImGui/**.cpp"
   }

   includedirs {
      "Source",

      -- Include Core
      "../Acro/Source",
      "../Vendor/GLAD/include",
      "../Vendor/GLFW/include",
      "../Vendor/glm",
      "../Vendor/stb_image",
      "../Vendor/ImGui"
   }

   targetdir ("../Binaries/" .. OutputDir .. "/%{prj.name}")
   objdir ("../Binaries/Intermediates/" .. OutputDir .. "/%{prj.name}")

   links { "Acro" }

   ----------------------------------
   -- WINDOWS CONFIGURATION
   ----------------------------------
   filter "system:windows"
      systemversion "latest"
      defines { "WINDOWS", "GLFW_STATIC" }
      libdirs { "../Vendor/GLFW/lib" }
      links { "glfw3", "opengl32.lib" }

   ----------------------------------
   -- LINUX CONFIGURATION (Hyprland)
   ----------------------------------
   filter "system:linux"
      defines { "LINUX", "GLFW_INCLUDE_NONE" }
      links { "glfw", "GL", "dl", "pthread", "wayland-client", "wayland-egl" }

      -- GLM is header-only, no need to link
      -- GLFW is installed via system package:
      -- sudo pacman -S glfw-wayland

   ----------------------------------
   -- DEBUG/RELEASE/DIST CONFIGS
   ----------------------------------
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

