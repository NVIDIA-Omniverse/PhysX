local ext_name = "omni.physx.camera"
local ext_dir = extsPhysics_dir.."/"..ext_name

define_physics_test_experience(ext_name)
define_etm_test_experience(ext_name)

group ("simulation")

repo_build.prebuild_copy {    
    { "python/__init__.py", ext_dir.."/omni/physxcamera/__init__.py" },
}

repo_build.prebuild_link {
    { "config", ext_dir.."/config" },
    { "data", ext_dir.."/data" },
    { "docs", ext_dir.."/docs" },
    { "python/scripts", ext_dir.."/omni/physxcamera/scripts" }
}

project ("omni.physx.camera.plugin")
    carbonitePlugin { ifaces = "%{root}/include/omni/physx", impl = "plugins", targetname = "omni.physx.camera.plugin" }
    staticruntime "Off"
    rtti "On"
    extension_usd_deps(targetDeps_dir, hostDeps_dir)
    extension_physxsdk_deps(targetDeps_dir, physxVersion)
    targetdir (targetDir.."/"..ext_dir.."/bin")
    dependson { "prebuild", "carb.physics-usd.plugin", "omni.physx.plugin" }
    includedirs {
        targetDeps_dir.."/carbonite/include",             
        kit_sdk_includes,
        targetDeps_dir.."/client-library/include",  -- to use UsdUtils.h
        runtime_include_dir }
    links { "physxSchema", "omni.usd", "foundation" }
    filter { "configurations:debug" }
        runtime "Debug"
    filter  { "configurations:release" }
        runtime "Release"
        if omniPhysXCameraOptimize then
            optimize "Speed"
        else
            optimize "off"
            runtime "Release"
            print("omni.physx.camera: Disabling optimization for release.")            
        end    
    filter { "system:windows" }
        libdirs { targetDeps_dir.."/assimp/Lib/x64", }
    filter { "system:linux" }
        exceptionhandling "On"
    filter {}
    files { "python/**.py" }
    vpaths { ['python/*'] = "python/**.py" }

project ("omni.physx.camera.python")
    carboniteBindingsPython {
        name = "_physxCamera",
        folder = "bindings",
        namespace = "omni" 
    }
    targetdir (targetDir.."/"..ext_dir.."/omni/physxcamera/bindings")
    includedirs {
        runtime_include_dir,
    }
