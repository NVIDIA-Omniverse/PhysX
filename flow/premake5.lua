
workspace "nvflow"
	configurations { "debug", "release" }
	filter { "system:linux" }
		platforms {"x86_64", "aarch64"}
	filter { "system:windows" }
		platforms {"x86_64"}
	filter {}

	startproject "nvfloweditor"

	local platform = "%{cfg.system}-%{cfg.platform}"
	local workspaceDir = "_compiler/" .. _ACTION
	local targetDir = "_build/" .. platform .. "/%{cfg.buildcfg}"
	local generatedDir = "_generated/" .. platform .. "/%{cfg.buildcfg}"

	location(workspaceDir)
	targetdir(targetDir)
	objdir("_build/intermediate/" ..platform.. "/%{prj.name}")

	staticruntime "On"
	cppdialect "C++11"
	exceptionhandling "Off"
	rtti "Off"
	flags { "FatalCompileWarnings", "NoPCH", "NoIncrementalLink" }

	floatingpoint "Fast"

	includedirs { "shared", "include/nvflow", "include/nvflow/shaders", generatedDir }

	filter { "platforms:x86_64" }
		architecture "x86_64"
	filter { "system:linux", "platforms:aarch64" }
		architecture "aarch64"

	filter { "configurations:debug" }
		optimize "Off"
		symbols "On"
		defines { "_DEBUG" }

	filter { "configurations:release" }
		optimize "On"
		symbols "On"
		defines  { "NDEBUG" }

	filter { "system:windows" }
		systemversion "10.0.17763.0"

	filter { "system:linux", "platforms:x86_64"}
		buildoptions { "-msse4" }
	filter { "system:linux" }
		linkoptions { "-Wl,-rpath,'$$ORIGIN',--no-as-needed", "-ldl", "-lpthread" }

	filter { }

function copy_to_targetdir(filePath)
	local relativeTargetDir = "../../../" .. targetDir
	local relativeFilePath = "../../../" .. filePath
	postbuildcommands { "{COPY} \"" .. relativeFilePath .. "\" \"" .. relativeTargetDir .. "\"" }
end

function prebuildShaderTool(shaderProjectFile)
	local relativeTargetDir = "../../../" .. targetDir
	local generatedPath = "../../../" .. generatedDir
	local shaderProjectPath = "../../../" .. shaderProjectFile
	filter { "system:windows" }
		prebuildcommands { "\"" .. relativeTargetDir .. "/nvflowshadertool.exe\" \"" .. generatedPath .. "\" \"" .. shaderProjectPath .. "\"" }
	filter { "system:linux" }
		prebuildcommands { "\"" .. relativeTargetDir .. "/nvflowshadertool\" \"" .. generatedPath .. "\" \"" .. shaderProjectPath .. "\"" }
	filter{}
end

function postbuildShaderTool()
	local generatedPath = "../../../" .. generatedDir .. "/%{prj.name}"
	postbuildcommands { "{MOVE} \"" .. generatedPath .. "/generated.cpp\" \"" .. generatedPath .. "/generatedOld.cpp\"" }
end

function addSourceDir(path)
	files
	{
		"include/%{prj.name}/*.h",
		"include/%{prj.name}/shaders/*.h",
		"include/%{prj.name}/shaders/*.hlsli",
		"shared/*.cpp",
		"shared/*.h",
		path .. "/*.cpp",
		path .. "/*.c",
		path .. "/*.h",
		path .. "/shaders/*.hlsl",
		path .. "/shaders/*.hlsli",
		path .. "/shaders/*.h",
		generatedDir .. "/%{prj.name}/generated.cpp",
		generatedDir .. "/%{prj.name}/generated_cpu.cpp",
		generatedDir .. "/%{prj.name}/*.h",
	}
	filter { "files:**.hlsl" }
		flags {"ExcludeFromBuild"}
	filter{}
end

project "nvfloweditor"
	kind "ConsoleApp"
	--dependson { "nvflowext", "nvflow", "nvflowshadertool" }
	prebuildShaderTool("source/%{prj.name}/NvFlowEditor.nfproj")
	postbuildShaderTool()
	location(workspaceDir .. "/%{prj.name}")
	language "C++"
	includedirs { generatedDir .. "/%{prj.name}", "include/nvflowext", "external/glfw/include", "external/imgui" }
	addSourceDir("source/%{prj.name}")
	addSourceDir("external/imgui")
	filter { "system:windows" }
		copy_to_targetdir("external/glfw/win64/glfw3.dll")
	filter { "system:linux" }
		copy_to_targetdir("external/glfw/linux/libglfw.so")
		copy_to_targetdir("external/glfw/linux/libglfw.so.3")
		copy_to_targetdir("external/glfw/linux/libglfw.so.3.3")
		copy_to_targetdir("external/glfw/linux/libglfw_aarch64.so.3.3")
	filter { }
