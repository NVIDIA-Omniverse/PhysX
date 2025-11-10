local ext_name = "omni.usdphysics"
local ext_dir = extsPhysics_dir.."/"..ext_name

define_physics_test_experience(ext_name)
define_etm_test_experience(ext_name)

group ("simulation")

repo_build.prebuild_copy {
    { "python/__init__.py", ext_dir.."/omni/usdphysics/__init__.py" },
}

repo_build.prebuild_link {
    { "config", ext_dir.."/config" },
    { "data", ext_dir.."/data" },
    { "docs", ext_dir.."/docs" },
    { "python/scripts", ext_dir.."/omni/usdphysics/scripts" },
}

project ("omni.usdphysics.plugin")
    carbonitePlugin { ifaces = "%{root}/include/omni/physics/schema", impl = "plugins", targetname = "omni.usdphysics.plugin" }
    staticruntime "Off"
    rtti "On"
    extension_usd_deps(targetDeps_dir, hostDeps_dir)
    targetdir (targetDir.."/"..ext_dir.."/bin/")
    dependson { "prebuild" }
    includedirs {
        targetDeps_dir.."/carbonite/include",
    }
    links { "physicsSchemaTools", "foundation" }
    filter { "configurations:debug" }
        runtime "Debug"
    filter  { "configurations:release" }
        runtime "Release"
        if omniUsdPhysicsOptimize then
            optimize "Speed"
        else
            optimize "off"
            runtime "Release"
            print("omni.usd.physics: Disabling optimization for release.")
        end
    link_boost_for_windows_wdefault()
    filter { "system:linux" }
        exceptionhandling "On"
    filter {}
    files { "python/**.py" }
    vpaths { ['python/*'] = "python/**.py" }

