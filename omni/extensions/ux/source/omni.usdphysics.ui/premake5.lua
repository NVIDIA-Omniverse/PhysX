local ext_name = "omni.usdphysics.ui"
local ext_dir = extsPhysics_dir.."/"..ext_name

define_physics_test_experience(ext_name)
define_etm_test_experience(ext_name)

group ("simulation")

repo_build.prebuild_copy {
    { "python/__init__.py", ext_dir.."/omni/usdphysicsui/__init__.py" }
}

repo_build.prebuild_link {
    { "config", ext_dir.."/config" },
    { "data", ext_dir.."/data" },
    { "docs", ext_dir.."/docs" },
    { "python/scripts", ext_dir.."/omni/usdphysicsui/scripts" },   
    { repo_root_dir.."/data/icons/omni.usdphysics.ui", ext_dir.."/icons" }, 
}

project ("omni.usdphysicsui.plugin")
    carbonitePlugin { ifaces = "%{root}/include/omni/physics/ui", impl = "plugins", targetname = "omni.usdphysicsui.plugin" }
    staticruntime "Off"
    rtti "On"
    extension_usd_deps(targetDeps_dir, hostDeps_dir)
    extension_imgui_deps()
    targetdir (targetDir.."/"..ext_dir.."/bin/")
    dependson { "prebuild", "carb.physics-usd.plugin", "foundation" }
    language "C++"
    includedirs {
        targetDeps_dir.."/carb_sdk_plugins/include",
        kit_sdk_includes,
        targetDeps_dir.."/client-library/include",
        kit_sdk_dir.."/dev/fabric/include",
        targetDeps_dir.."/gsl/include",   -- Support for std::span
        runtime_include_dir,
    }
    links { "omni.usd", "foundation", "carb" }

    filter { "configurations:debug" }
        runtime "Debug"
    filter  { "configurations:release" }
        runtime "Release"
        if omniUsdPhysicsUiOptimize then
            optimize "Speed"
        else
            optimize "off"
            runtime "Release"
            print("omni.usdphysicsui: Disabling optimization for release.")
        end
    filter { "system:linux" }
        exceptionhandling "On"
        links { "rt" }
    filter {}
    files { "python/**.py" }
    vpaths { ['python/*'] = "python/**.py" }


project ("omni.usdphysicsui.python")
    carboniteBindingsPython {
        name = "_usdphysicsUI",
        folder = "bindings",
        namespace = "omni" }
    extension_usd_deps(targetDeps_dir, hostDeps_dir)
    includedirs {
        kit_sdk_includes,
        
        runtime_include_dir
    }
    filter  { "configurations:release" }
        runtime "Release"
        if omniUsdPhysicsUiOptimize then
            optimize "Speed"
        else
            optimize "off"
            runtime "Release"
            print("omni.usdphysicsui.python: Disabling optimization for release.")
        end
    filter {}
    targetdir (targetDir.."/"..ext_dir.."/omni/usdphysicsui/bindings")
    link_boost_for_windows_wdefault()

