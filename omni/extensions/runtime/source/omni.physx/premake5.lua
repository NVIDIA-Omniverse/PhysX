local ext_name = "omni.physx"
local ext_dir = extsPhysics_dir.."/"..ext_name

define_physics_test_experience(ext_name)
define_etm_test_experience(ext_name)

group ("simulation")

repo_build.prebuild_copy {
    { "python/__init__.py", ext_dir.."/omni/physx/__init__.py" },
}

repo_build.prebuild_link {
    { "config", ext_dir.."/config" },
    { "data", ext_dir.."/data" },
    { "docs", ext_dir.."/docs" },
    { "python/scripts", ext_dir.."/omni/physx/scripts" },
}

project ("omni.physx.plugin")
    carbonitePlugin { ifaces = "%{root}/include/omni/physx", impl = "plugins", targetname = "omni.physx.plugin" }
    staticruntime "Off"
    rtti "On"
    targetdir (targetDir.."/"..ext_dir.."/bin")
    if os.target() == "windows" and _OPTIONS["devphysx"] then
        fastuptodate "Off"
        dependson { "prebuild", "carb.physics-usd.plugin", "foundation", "PhysX", 
        "PhysXExtensions", "PhysXGpu", "PhysXVehicle2", "PhysXCharacterKinematic", "PVDRuntime", }
    else
        dependson { "prebuild", "carb.physics-usd.plugin", "foundation" }
    end
    extension_usd_deps(targetDeps_dir, hostDeps_dir)
    extension_physxsdk_deps(targetDeps_dir, physxVersion)
    add_cuda_deps(targetDeps_dir)
    includedirs {
        targetDeps_dir.."/carbonite/include",
        kit_sdk_dir.."/dev/fabric/include",
        targetDeps_dir.."/client-library/include",
        targetDeps_dir.."/gsl/include",   -- Support for std::span
        targetDeps_dir.."/"..physxVersion.."/pvdruntime/include",
        "plugins"
    }

    -- Enable AVX2 instruction set
    filter { "system:windows" }
        buildoptions { "/arch:AVX" }
    filter { "system:linux", "platforms:x86_64" }
        buildoptions { "-mavx" }

    filter { "configurations:debug" }
        runtime "Debug"
    filter  { "configurations:release" }
        runtime "Release"
        if omniPhysXOptimize then
            optimize "Speed"
        else
            optimize "off"
            runtime "Release"
            print("omni.physx: Disabling optimization for release.")            
        end
    filter { "system:windows" }
        libdirs { targetDeps_dir.."/assimp/Lib/x64",
                    }
        links { "physicsSchemaTools", "physxSchema", "foundation", "carb" }
    filter { "system:windows", "platforms:x86_64", "configurations:debug" }
        repo_build.copy_to_targetdir(root.."/_build/target-deps/"..physxVersion..'/bin/win.x86_64.vc142.md/debug/PVDRuntime_64.dll')
    filter { "system:windows", "platforms:x86_64", "configurations:release" }
        repo_build.copy_to_targetdir(root.."/_build/target-deps/"..physxVersion..'/bin/win.x86_64.vc142.md/'..physxLibs..'/PVDRuntime_64.dll')
    filter { "system:linux" }
        exceptionhandling "On"
        removeflags { "FatalCompileWarnings", "UndefinedIdentifiers" }
        libdirs {
            targetDeps_dir.."/usd/%{cfg.buildcfg}/lib",
            }
        links { "physicsSchemaTools", "physxSchema", "foundation", "carb" }
    filter { "system:linux", "platforms:x86_64", "configurations:debug" }
        repo_build.copy_to_targetdir(root.."/_build/target-deps/"..physxVersion..'/bin/linux.x86_64/debug/libPVDRuntime_64.so')
     filter { "system:linux", "platforms:x86_64", "configurations:release" }
        repo_build.copy_to_targetdir(root.."/_build/target-deps/"..physxVersion..'/bin/linux.x86_64/'..physxLibs..'/libPVDRuntime_64.so')
    filter {}
    files { "python/**.py" }
    vpaths { ['python/*'] = "python/**.py" }

project ("omni.physx.python")
    carboniteBindingsPython {
        name = "_physx",
        folder = "bindings",
        namespace = "omni" }
    extension_usd_deps(targetDeps_dir, hostDeps_dir)
    links { "physicsSchemaTools", "physxSchema" }
    includedirs {
        targetDeps_dir.."/carbonite/include",
    }
    targetdir (targetDir.."/"..ext_dir.."/omni/physx/bindings")
    link_boost_for_windows_wdefault()

