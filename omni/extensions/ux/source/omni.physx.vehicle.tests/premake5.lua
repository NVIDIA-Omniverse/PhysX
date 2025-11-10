local ext_name = "omni.physx.vehicle.tests"
local ext_dir = extsPhysics_dir.."/"..ext_name

define_physics_test_experience(ext_name)

group ("simulation")

repo_build.prebuild_copy {    
    { "python/__init__.py", ext_dir.."/omni/physxvehicletests/__init__.py" },
}

repo_build.prebuild_link {
    { "config", ext_dir.."/config" },
    { "python/scripts", ext_dir.."/omni/physxvehicletests/scripts" },
    { repo_root_dir.."/data/audio/physics/vehicle", ext_dir.."/data/audio" },
    { repo_root_dir.."/data/usd/tests/Physics/Vehicle_Schema_Tests", ext_dir.."/data/tests/Vehicle_Schema_Tests" },
}

project ("omni.physx.vehicle.tests.plugin")
    carbonitePlugin { ifaces = "%{root}/include/private/omni/physx", impl = "plugins", targetname = "omni.physx.vehicle.tests.plugin" }
    staticruntime "Off"
    rtti "On"
    extension_usd_deps(targetDeps_dir, hostDeps_dir)
    extension_physxsdk_deps(targetDeps_dir, physxVersion)
    targetdir (targetDir.."/"..ext_dir.."/bin")
    dependson { "prebuild", "carb.physics-usd.plugin", "omni.physx.plugin", "omni.physx.vehicle.plugin" }
    includedirs {
        targetDeps_dir.."/carbonite/include",        
        kit_sdk_includes,
        runtime_include_dir }
    filter { "system:windows" }
        libdirs { targetDeps_dir.."/assimp/Lib/x64", }
        links { "physxSchema", "omni.usd" }
    filter { "system:linux" }
        exceptionhandling "On"
        links { "physxSchema", "omni.usd" }   
    filter {}
    files { "python/**.py" }
    vpaths { ['python/*'] = "python/**.py" }

project ("omni.physx.vehicle.tests.python")
    carboniteBindingsPython {
        name = "_physxVehicleTests",
        folder = "bindings",
        namespace = "omni" 
    }
    includedirs { runtime_include_dir }
    targetdir (targetDir.."/"..ext_dir.."/omni/physxvehicletests/bindings")
