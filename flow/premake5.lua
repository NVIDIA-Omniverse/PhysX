workspace "nvflow"
    configurations { "debug", "release" }
    filter { "system:linux" }
        platforms {"x86_64", "aarch64"}
    filter { "system:windows" }
        platforms {"x86_64"}
    filter {}

    startproject "nvfloweditor"

    local platform = "%{cfg.system}-%{cfg.platform}"
    local workspaceDir = "_compiler/" .. (_ACTION or "build")
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
    disablewarnings { "4550" }

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
        local msvc_toolset = "./external/msvc/VC/Tools/MSVC/14.29.30133"
        local msvc_toolset_env = os.getenv("MSVS_TOOLSET")
        if msvc_toolset_env then
            msvc_toolset = msvc_toolset_env
        end

        local ms_build_dir = "./external/msvc/MSBuild/Current/bin"
        local ms_build_dir_env = os.getenv("MSBUILD_DIR")
        if ms_build_dir_env then
            ms_build_dir = ms_build_dir_env
        end

        local msvcInclude = msvc_toolset.."/include"

        local winsdk = "./external/winsdk"
        local winsdkInclude = winsdk.."/include"
        local winsdkIncludes = {
            winsdkInclude.."/winrt",
            winsdkInclude.."/um",
            winsdkInclude.."/ucrt",
            winsdkInclude.."/shared"
        }

        externalincludedirs { msvcInclude, winsdkIncludes }
        local msvcLibs = msvc_toolset.."/lib/onecore/x64"
        local sdkLibs = { winsdk.."/lib/ucrt/x64", winsdk.."/lib/um/x64" }
        syslibdirs { msvcLibs, sdkLibs }
        bindirs {
            msvc_toolset.."/bin/HostX64/x64",
            ms_build_dir,
            winsdk.."/bin/x64"
        }

        if os.getenv("SLANG_DEBUG_OUTPUT") then
            defines { "SLANG_DEBUG_OUTPUT" }
        end

    filter { "system:linux", "platforms:x86_64"}
        buildoptions { "-msse4" }
    filter { "system:linux" }
        buildoptions { "-fvisibility=hidden " }
        linkoptions { "-Wl,-rpath,'$$ORIGIN',--no-as-needed", "-ldl", "-lpthread" }

    filter { }

-- NvFlowShaderCompiler

function addSourceDirTool(path)
    files
    {
        "shared/*.cpp",
        "shared/*.h",
        path .. "/*.cpp",
        path .. "/*.h",
    }
end

function copy_to_targetdir(filePath)
    local relativeTargetDir = "../../../" .. targetDir
    local relativeFilePath = "../../../" .. filePath
    postbuildcommands { "{COPY} \"" .. relativeFilePath .. "\" \"" .. relativeTargetDir .. "\"" }
end

project "nvflowshadercompiler"
    kind "ConsoleApp"
    filter { "configurations:release" }
        optimize "On"
    filter {}
    location(workspaceDir .. "/%{prj.name}")
    language "C++"
    addSourceDirTool("source/nvflowshadercompiler")

    local relativeTargetDir = "../../../" .. targetDir
    local relativeExternalDir = "../../../external"

    includedirs { "external/slang/include" }

    prebuildcommands {"{MKDIR} " .. relativeTargetDir }

    filter { "system:windows" }
        prebuildcommands { "{COPY} " .. relativeExternalDir .. "/slang/bin/slang.dll ".. relativeTargetDir }
        prebuildcommands { "{COPY} " .. relativeExternalDir .. "/slang/bin/slang-glslang.dll ".. relativeTargetDir }
    filter { "system:linux" }
        prebuildcommands { "{COPY} " .. relativeExternalDir .. "/slang/lib/libslang.so ".. relativeTargetDir }
        prebuildcommands { "{COPY} " .. relativeExternalDir .. "/slang/lib/libslang-glslang.so ".. relativeTargetDir }
    filter{}

    libdirs { "external/slang/lib" }
    links { "slang" }

-- NvFlowShaderTool

project "nvflowshadertool"
    kind "ConsoleApp"
    dependson { "nvflowshadercompiler" }
    filter { "configurations:release" }
        optimize "On"
    filter {}
    location(workspaceDir .. "/%{prj.name}")
    language "C++"
    addSourceDirTool("source/nvflowshadertool")

-- NvFlow libraries

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

function addSourceDir(path, source_project)
    files
    {
        "include/".. source_project .. "/*.h",
        "include/".. source_project .. "/shaders/*.h",
        "include/".. source_project .. "/shaders/*.hlsli",
        "include/".. source_project .. "/nanovdb/*.h",
        "include/".. source_project .. "/nanovdb/putil/*.h",
        "shared/*.cpp",
        "shared/*.h",
        path .. "/*.cpp",
        path .. "/*.c",
        path .. "/*.h",
        path .. "/shaders/*.hlsl",
        path .. "/shaders/*.hlsli",
        path .. "/shaders/*.h",
        generatedDir .. "/" .. source_project .. "/*.h",
    }
    filter { "files:**.hlsl" }
        flags {"ExcludeFromBuild"}
    filter{}
end

project "nvflow"
    kind "SharedLib"
    dependson { "nvflowshadertool" }
    prebuildShaderTool("source/nvflow/NvFlow.nfproj")
    location(workspaceDir .. "/%{prj.name}")
    language "C++"
    includedirs { generatedDir .. "/nvflow" }
    addSourceDir("source/nvflow", "nvflow")

project "nvflow_rtx"
    kind "SharedLib"
    dependson { "nvflow", "nvflowshadertool" }
    location(workspaceDir .. "/%{prj.name}")
    language "C++"
    includedirs { generatedDir .. "/nvflow" }
    addSourceDir("source/nvflow", "nvflow")

project "nvflowext"
    kind "SharedLib"
    dependson { "nvflow_rtx", "nvflowshadertool" }
    prebuildShaderTool("source/nvflowext/NvFlowExt.nfproj")
    location(workspaceDir .. "/%{prj.name}")
    language "C++"
    includedirs { generatedDir .. "/nvflowext", "include/nvflowext","include/nvflowext/shaders", "external/vulkan", "external/cuda/include" }
    addSourceDir("source/nvflowext", "nvflowext")
    files
    {
        "source/nvflowext/vulkan/*.cpp",
        "source/nvflowext/vulkan/*.h",
        "source/nvflowext/cpu/*.cpp",
        "source/nvflowext/cpu/*.h",
        "source/nvflowext/opt/*.cpp",
        "source/nvflowext/opt/*.h",
    }

project "nvflowext_rtx"
    kind "SharedLib"
    dependson { "nvflowext", "nvflowshadertool" }
    location(workspaceDir .. "/%{prj.name}")
    language "C++"
    includedirs { generatedDir .. "/nvflowext", "include/nvflowext","include/nvflowext/shaders", "external/vulkan", "external/cuda/include" }
    addSourceDir("source/nvflowext", "nvflowext")
    files
    {
        "source/nvflowext/vulkan/*.cpp",
        "source/nvflowext/vulkan/*.h",
        "source/nvflowext/cpu/*.cpp",
        "source/nvflowext/cpu/*.h",
        "source/nvflowext/opt/*.cpp",
        "source/nvflowext/opt/*.h",
    }

project "nvfloweditor"
    kind "ConsoleApp"
    dependson { "nvflowext", "nvflow", "nvflowshadertool" }
    prebuildShaderTool("source/%{prj.name}/NvFlowEditor.nfproj")
    location(workspaceDir .. "/%{prj.name}")
    language "C++"
    includedirs { generatedDir .. "/nvfloweditor", "include/nvflowext", "external/glfw/include", "external/imgui" }
    addSourceDir("source/%{prj.name}", "nvfloweditor")
    addSourceDir("external/imgui", "nvfloweditor")
    filter { "system:windows" }
        copy_to_targetdir("external/glfw/win64/glfw3.dll")
    filter { "system:linux", "platforms:x86_64" }
        copy_to_targetdir("external/glfw/linux/libglfw.so")
        copy_to_targetdir("external/glfw/linux/libglfw.so.3")
        copy_to_targetdir("external/glfw/linux/libglfw.so.3.3")
    filter { "system:linux", "platforms:aarch64" }
        copy_to_targetdir("external/glfw/linux/libglfw_aarch64.so.3.3")
    filter { }
