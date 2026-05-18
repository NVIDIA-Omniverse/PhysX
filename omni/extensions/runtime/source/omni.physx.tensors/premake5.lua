local ext_name = "omni.physx.tensors"
local ext_dir = extsPhysics_dir.."/"..ext_name

define_physics_test_experience(ext_name)
define_etm_test_experience(ext_name)

group ("simulation")

repo_build.prebuild_link {
    { "config", ext_dir.."/config" },
    { "data", ext_dir.."/data" },
    { "docs", ext_dir.."/docs" },
}

project ("omni.physx.tensors.plugin")
    carbonitePlugin { ifaces = "%{root}/include/omni/physics/tensors", impl = "plugins", targetname = "omni.physx.tensors.plugin" }
    staticruntime "Off"
    rtti "On"
    extension_usd_deps(targetDeps_dir, hostDeps_dir)
    extension_physxsdk_deps(targetDeps_dir, physxVersion)
    targetdir (targetDir.."/"..ext_dir.."/bin")
    dependson { "prebuild", "carb.physics-usd.plugin", "omni.physx.plugin" }
    includedirs {
        targetDeps_dir.."/carb_sdk_plugins/include",
        kit_sdk_includes,
        kit_sdk_dir.."/dev/fabric/include",
        targetDeps_dir.."/gsl/include",
    }
    links { "physxSchema", "physicsSchemaTools", "foundation", "carb" }
    filter { "configurations:debug" }
        runtime "Debug"
    filter  { "configurations:release" }
        runtime "Release"
        if omniPhysXTensorsOptimize then
            optimize "Speed"
        else
            optimize "off"
            runtime "Release"
            print("omni.physx.tensors: Disabling optimization for release.")            
        end    
    filter { "system:windows" }
        libdirs { targetDeps_dir.."/assimp/Lib/x64", }
    filter { "system:linux" }
        exceptionhandling "On"
        links { "rt" }
    filter {}

    add_files("iface", extensions_root_dir.."/include/omni/physics/tensors")
    add_files("impl", "plugins")

    add_cuda_dirs()
    add_nvcc_commands()

    -- NOTE: "cuda" intentionally NOT linked -- all cu* driver API calls go through
    -- IOptionalCuda (runtime-loaded shim in foundation). This eliminates the
    -- DT_NEEDED libcuda.so.1 entry, allowing the plugin to load on CPU-only machines.
    -- cudart_static is a static archive (.a) that embeds without creating DT_NEEDED.
    links { "cudart_static" }
