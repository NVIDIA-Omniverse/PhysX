
local ext_name = "omni.physx.vehicle"
local ext_dir = extsPhysics_dir.."/"..ext_name
local ext_dir_subpath = "omni/physxvehicle"
local ext_dir_with_subpath = ext_dir.."/"..ext_dir_subpath

define_physics_test_experience(ext_name)
define_etm_test_experience(ext_name)

group ("simulation")

repo_build.prebuild_copy {    
    { "python/__init__.py", ext_dir_with_subpath.."/__init__.py" },
}

repo_build.prebuild_link {
    { "config", ext_dir.."/config" },
    { "data", ext_dir.."/data" },
    { "docs", ext_dir.."/docs" },
    { "python/scripts", ext_dir_with_subpath.."/scripts" },    
}

project ("omni.physx.vehicle.plugin")
    carbonitePlugin { ifaces = "%{root}/include/omni/physx", impl = "plugins", targetname = "omni.physx.vehicle.plugin" }
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
        runtime_include_dir
    }     
    links { "physxSchema", "omni.usd", "foundation" }
    filter { "configurations:debug" }
        runtime "Debug"
    filter  { "configurations:release" }
        runtime "Release"
        if omniPhysXVehicleOptimize then
            optimize "Speed"
        else
            optimize "off"
            runtime "Release"
            print("omni.physx.vehicle: Disabling optimization for release.")            
        end    
    filter { "system:windows" }
        libdirs { targetDeps_dir.."/assimp/Lib/x64", }
    filter { "system:linux" }
        exceptionhandling "On"
    filter {}
    files { "python/**.py" }
    vpaths { ['python/*'] = "python/**.py" }

project ("omni.physx.vehicle.python")
    carboniteBindingsPython {
        name = "_physxVehicle",
        folder = "bindings",
        namespace = "omni" 
    }
    includedirs { runtime_include_dir }
    targetdir (targetDir.."/"..ext_dir_with_subpath.."/bindings")
