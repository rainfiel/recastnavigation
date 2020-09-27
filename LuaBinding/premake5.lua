--
-- premake5 file to build RecastDemo
-- http://premake.github.io/
--

local action = _ACTION or ""
local todir = "Build/" .. action

solution "LuaBinding"
	configurations { 
		"Debug",
		"Release"
	}

	location (todir)

	floatingpoint "Fast"
	symbols "On"
	exceptionhandling "Off"
	rtti "Off"
	flags { "FatalCompileWarnings" }

	-- debug configs
	configuration "Debug*"
		defines { "DEBUG" }
		targetdir ( todir .. "/lib/Debug" )
 
 	-- release configs
	configuration "Release*"
		defines { "NDEBUG" }
		optimize "On"
		targetdir ( todir .. "/lib/Release" )

	configuration "not windows"
		warnings "Extra"

	-- windows specific
	configuration "windows"
		platforms { "Win32", "Win64" }
		defines { "WIN32", "_WINDOWS", "_CRT_SECURE_NO_WARNINGS", "_HAS_EXCEPTIONS=0", "LUA_BUILD_AS_DLL", "LUA_BINDING" }
		-- warnings "Extra" uses /W4 which is too aggressive for us, so use W3 instead.
		-- Disable:
		-- * C4351: new behavior for array initialization
		buildoptions { "/W3", "/wd4351" }

	filter "platforms:Win32"
		architecture "x32"

	filter "platforms:Win64"
		architecture "x64"

project "Lua"
	language "C"
	kind "SharedLib"
	includedirs { 
	}
	files { 
		"../Lua/*.h",
		"../Lua/*.c" 
	}

project "LuaBinding"
	language "C++"
	kind "SharedLib"
	includedirs { 
		"../LuaBinding/Include",
		"../DebugUtils/Include",
		"../Detour/Include",
		"../DetourCrowd/Include",
		"../DetourTileCache/Include",
		"../Recast/Include",
		"../RecastDemo/Include",
		"../Lua"
	}
	files { 
		"../DebugUtils/Include/*.h",
		"../DebugUtils/Source/*.cpp",
		"../Detour/Include/*.h", 
		"../Detour/Source/*.cpp",
		"../DetourCrowd/Include/*.h",
		"../DetourCrowd/Source/*.cpp",
		"../DetourTileCache/Include/*.h",
		"../DetourTileCache/Source/*.cpp",
		"../Recast/Include/*.h",
		"../Recast/Source/*.cpp", 
		"../RecastDemo/Source/ChunkyTriMesh.cpp",
		"../RecastDemo/Source/InputGeom.cpp",
		"../RecastDemo/Source/MeshLoaderObj.cpp",
		"../RecastDemo/Source/PerfTimer.cpp",
		"../RecastDemo/Source/SampleInterfaces.cpp",
		"../LuaBinding/Include/*.h",
		"../LuaBinding/Source/*.cpp",
		"../LuaBinding/Source/*.c" 
	}
	-- project dependencies
	links { 
		"Lua"
	}
	targetdir "Bin"

	-- linux library cflags and libs
	configuration { "linux", "gmake" }
		buildoptions { 
			"-Wno-class-memaccess"
		}
