local lib_dir = workspaceDir.."/%{prj.name}".."/lib".."/%{cfg.buildcfg}"

project ("foundation")
    kind "StaticLib"
    staticruntime "Off"
    location (workspaceDir.."/%{prj.name}")
    rtti "On"
    targetdir (lib_dir)
    dependson { "prebuild", "physxSchema", "physxSchemaTools", "_physxSchema", "_physxSchemaTools" }
    includedirs {
        "../pch",
        repo_root_dir.."/include/extras",
        targetDeps_dir.."/carbonite/include",
        "include/common/foundation",
        "include/common/utilities",
        "include/common/ui",
        "include/",
        runtime_include_dir,
        "../../include",
        targetDeps_dir.."/physx/include",
        schema_package_dir.."/include",
        targetDeps_dir.."/usd/%{cfg.buildcfg}/include",          
        targetDeps_dir.."/rtx_plugins/include",
        targetDeps_dir.."/python/include",
    }
    extension_imgui_deps()
    filter { "system:linux" }
        exceptionhandling "On"
        includedirs { targetDeps_dir.."/usd/%{cfg.buildcfg}/include/boost",
                    targetDeps_dir.."/python/include/"..PYTHON_LIB }
        removeflags { "UndefinedIdentifiers" }
        buildoptions { "-fPIC" }
    filter {}
    files { "**.h", "**.cpp" }
    vpaths {
        ['include/**'] = "include/common/**.h",
        ['source/*'] = { "source/foundation/**.h", "source/foundation/**.cpp", "source/utilities/**.h", "source/utilities/**.cpp" },
    }
