local ext_name = "omni.physx.cooking"
local ext_dir = extsPhysics_dir.."/"..ext_name

define_physics_test_experience(ext_name)
define_etm_test_experience(ext_name)

-- define another test that sets the remote cache flag for UJITSO
local remote_cache_args = {
    test_args = {
        "--",
        "--/physics/cooking/ujitsoRemoteCacheEnabled=true",
        "--/UJITSO/datastore/GRPCDataStore/selfFile/enabled=true",
        "--/UJITSO/datastore/allowGRPCDataStore=true",
        "--/UJITSO/datastore/GRPCDataStore/useTLS=false",
        "--/UJITSO/datastore/GRPCDataStore/tenantID=abcdefg-hijklmnopqrstuvwxyzAB_CDEFGHIJKLMNO",
        "--/UJITSO/datastore/GRPCDataStoreServer/cachePath=\"__temppath__\"",
        "--/UJITSO/datastore/GRPCDataStoreServer/address=0.0.0.0:7890",
        "--/UJITSO/datastore/grpcDnsName=0.0.0.0:7890",
    },
    suffix=".remote_cache"
}
define_physics_test_experience(ext_name, remote_cache_args)

group ("simulation")

repo_build.prebuild_copy {
    { "python/__init__.py", ext_dir.."/omni/physxcooking/__init__.py" },
}

repo_build.prebuild_link {
    { "config", ext_dir.."/config" },
    { "data", ext_dir.."/data" },
    { "docs", ext_dir.."/docs" },
    { "python/scripts", ext_dir.."/omni/physxcooking/scripts" },
    { "python/tests", ext_dir.."/omni/physxcooking/tests" },
}

project ("omni.physx.cooking.plugin")
    carbonitePlugin { ifaces = "%{root}/include/omni/physx", impl = "plugins", targetname = "omni.physx.cooking.plugin" }
    staticruntime "Off"
    rtti "On"

    targetdir (targetDir.."/"..ext_dir.."/bin")
    if os.target() == "windows" and _OPTIONS["devphysx"] then
        dependson { "prebuild", "carb.physics-usd.plugin", "foundation", "carb",
                    "PhysX", "PhysXExtensions", "PhysXGpu", "PhysXVehicle", "PhysXCharacterKinematic"}
    else
        dependson { "prebuild", "carb.physics-usd.plugin", "foundation", "carb" }
    end

    extension_usd_deps(targetDeps_dir, hostDeps_dir)
    extension_physxsdk_deps(targetDeps_dir, physxVersion)
    add_cuda_deps(targetDeps_dir)
    includedirs {
        targetDeps_dir.."/gsl/include",   -- Support for gsl::span
        targetDeps_dir.."/client-library/include",
    }
    filter { "configurations:debug" }
        runtime "Debug"
    filter  { "configurations:release" }
        runtime "Release"
        if omniPhysXCookingOptimize then
            optimize "Speed"
        else
            optimize "off"
            runtime "Release"
            print("omni.physx.cooking: Disabling optimization for release.")            
        end
    filter {}
    filter { "system:windows" }
        links { "physicsSchemaTools", "physxSchema", "foundation", "carb"}
    filter { "system:linux" }
        links { "physicsSchemaTools", "physxSchema", "foundation", "carb"}
        exceptionhandling "On"
    filter {}
    files { "python/**.py" }
    vpaths { ['python/*'] = "python/**.py" }

if not _OPTIONS["no-runtime-pybind"] then
    project ("omni.physx.cooking.python")
        carboniteBindingsPython {
            name = "_physxCooking",
            folder = "bindings",
            namespace = "omni" }
        extension_usd_deps(targetDeps_dir, hostDeps_dir)
        links {  "physxSchema" }
        includedirs {
            kit_sdk_includes,
        }
        targetdir (targetDir.."/"..ext_dir.."/omni/physxcooking/bindings")
        filter {}
end
