local ext_name = "omni.physx.fabric"
local ext_dir = extsPhysics_dir.."/"..ext_name

define_physics_test_experience(ext_name)
define_etm_test_experience(ext_name)

group ("simulation")

repo_build.prebuild_copy {        
    { "python/__init__.py", ext_dir.."/omni/physxfabric/__init__.py" },
}

repo_build.prebuild_link {
    { "config", ext_dir.."/config" },
    { "data", ext_dir.."/data" },
    { "docs", ext_dir.."/docs" },
    { "python/scripts", ext_dir.."/omni/physxfabric/scripts" },
    { "python/tests", ext_dir.."/omni/physxfabric/tests" },
}

project ("omni.physx.fabric.plugin")
    carbonitePlugin { ifaces = "%{root}/include/omni/physx", impl = "plugins", targetname = "omni.physx.fabric.plugin" }
    staticruntime "Off"
    rtti "On"
    extension_usd_deps(targetDeps_dir, hostDeps_dir)
    extension_physxsdk_deps(targetDeps_dir, physxVersion)
    targetdir (targetDir.."/"..ext_dir.."/bin")
    dependson { "prebuild", "omni.physx.plugin" }
    includedirs {
        targetDeps_dir.."/carb_sdk_plugins/include",
        targetDeps_dir.."/rtx_plugins/include",
        targetDeps_dir.."/client-library/include",
        targetDeps_dir.."/gsl/include",   -- Support for std::span 
        kit_sdk_dir.."/dev/fabric/include",
        kit_sdk_includes,
        
    }

    links { "cuda", "cudart_static", "carb" }

    filter { "configurations:debug" }
        runtime "Debug"
    filter  { "configurations:release" }
        runtime "Release"
        if omniPhysXFabricOptimize then
            optimize "Speed"
        else
            optimize "off"
            runtime "Release"
            print("omni.physx.fabric: Disabling optimization for release.")            
        end        
    filter { "system:windows" }
        libdirs { targetDeps_dir.."/assimp/Lib/x64" }
        links { "physicsSchemaTools", "physxSchema" }
    filter { "system:linux" }
        exceptionhandling "On"
        links { "physicsSchemaTools", "physxSchema", "rt" }
    filter {}
   
   
    add_cuda_dirs()
    add_nvcc_commands()

    files { "python/**.py" }
    vpaths { ['python/*'] = "python/**.py" }

project ("omni.physx.fabric.python")
    carboniteBindingsPython {
        name = "_physxFabric",
        folder = "bindings",
        namespace = "omni" 
    }
    targetdir (targetDir.."/"..ext_dir.."/omni/physxfabric/bindings")
