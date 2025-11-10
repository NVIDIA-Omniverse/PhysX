local ext_name = "omni.convexdecomposition"
local ext_dir = extsPhysics_dir.."/"..ext_name

define_physics_test_experience(ext_name)
define_etm_test_experience(ext_name)

group ("simulation")

repo_build.prebuild_copy {        
    { "python/__init__.py", ext_dir.."/omni/convexdecomposition/__init__.py" },
}

repo_build.prebuild_link {
    { "config", ext_dir.."/config" },
    { "data", ext_dir.."/data" },
    { "docs", ext_dir.."/docs" },
    { "python/scripts", ext_dir.."/omni/convexdecomposition/scripts" },
    { "python/tests", ext_dir.."/omni/convexdecomposition/tests" },
}

project "omni.convexdecomposition.plugin"    
    carbonitePlugin { ifaces = "%{root}/include/omni/convexdecomposition", impl = "plugins"}
    staticruntime "Off"
    exceptionhandling "On"
    targetdir (targetDir.."/"..ext_dir.."/bin")
    rtti "On"
    filter { "configurations:debug" }
        runtime "Debug"
    filter  { "configurations:release" }
        runtime "Release"
        if omniConvexdecompositionOptimize then
            optimize "Speed"
        else
            optimize "off"
            runtime "Release"
            print("omni.convexdecomposition: Disabling optimization for release.")            
        end
    filter {}
    extension_usd_deps(targetDeps_dir, hostDeps_dir)    
    includedirs {
        targetDeps_dir.."/carbonite/include",
        runtime_include_dir
    }
    dependson { "prebuild", "carb" }

    filter { "files:plugins/FM.cpp", "system:linux"}
        buildoptions { "-Wno-strict-aliasing" }
    filter {}
    links { "foundation" }

    filter {}
    files { "python/**.py" }
    vpaths { ['python/*'] = "python/**.py" }

    filter {}

project "omni.convexdecomposition.python"
    carboniteBindingsPython {
        name = "_convexdecomposition",
        folder = "bindings",
        namespace = "omni" }
    targetdir (targetDir.."/"..ext_dir.."/omni/convexdecomposition/bindings")
    includedirs {
        runtime_include_dir
    }