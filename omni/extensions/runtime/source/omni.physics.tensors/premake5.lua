local ext_name = "omni.physics.tensors"
local ext_dir = extsPhysics_dir.."/"..ext_name

define_physics_test_experience(ext_name)
define_etm_test_experience(ext_name)

group ("simulation"..physxTag)

repo_build.prebuild_copy {    
    { "bindings/python/__init__.py", ext_dir.."/omni/physics/tensors/__init__.py" },
}

repo_build.prebuild_link {
    { "config", ext_dir.."/config" },
    { "data", ext_dir.."/data" },
    { "docs", ext_dir.."/docs" },
    { "python", ext_dir.."/omni/physics/tensors/impl" },
}

project ("omni.physics.tensors.plugin")
    carbonitePlugin { ifaces = "%{root}/include/omni/physics/tensors", impl = "plugins", targetname = "omni.physics.tensors.plugin" }
    staticruntime "Off"
    rtti "On"
    extension_usd_deps(targetDeps_dir, hostDeps_dir)
    extension_physxsdk_deps(targetDeps_dir, physxVersion)
    targetdir (targetDir.."/"..ext_dir.."/bin")
    dependson { "prebuild", "carb.physics-usd.plugin", "omni.physx.plugin" }
    includedirs {
        targetDeps_dir.."/carb_sdk_plugins/include",        
    }    
    libdirs {
    }
    links { "physxSchema", "foundation" }
    filter { "configurations:debug" }
        runtime "Debug"
    filter  { "configurations:release" }
        runtime "Release"
        if omniPhysicsTensorsOptimize then
            optimize "Speed"
        else
            optimize "off"
            runtime "Release"
            print("omni.physics.tensors: Disabling optimization for release.")            
        end    
        
    add_files("iface", "%{root}/include/omni/physics/tensors")
    add_files("impl", "plugins")

    files { "python/**.py" }
    vpaths { ['python/*'] = "python/**.py" }

project ("omni.physics.tensors.python")
    carboniteBindingsPython {
        name = "_physicsTensors",
        folder = "bindings/python",
        namespace = "omni", -- ??
    }
    targetdir (targetDir.."/"..ext_dir.."/omni/physics/tensors/bindings")
