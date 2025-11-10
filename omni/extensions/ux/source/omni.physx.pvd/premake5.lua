local ext_name = "omni.physx.pvd"
local ext_dir = extsPhysics_dir.."/"..ext_name

define_physics_test_experience("omni.physx.pvd")
define_etm_test_experience(ext_name)

group ("simulation")

repo_build.prebuild_copy {    
    { "python/__init__.py", ext_dir.."/omni/physxpvd/__init__.py" },
}

repo_build.prebuild_link 
{
    { "config", ext_dir.."/config" },
	{  "docs", ext_dir.."/docs"  },
    {  "data", ext_dir.."/data"  },
    { "python/scripts", ext_dir.."/omni/physxpvd/scripts" },    
}

project ("omni.physx.pvd.plugin")
    carbonitePlugin { ifaces = "%{root}/include/omni/physx", impl = "plugins", targetname = "omni.physx.pvd.plugin" }
    staticruntime "Off"
    rtti "On"
    extension_usd_deps(targetDeps_dir, hostDeps_dir)
    extension_physxsdk_deps(targetDeps_dir, physxVersion)
    targetdir (targetDir.."/"..ext_dir.."/bin")
    dependson { "prebuild", "carb.physics-usd.plugin", "omni.physx.plugin" }
    includedirs {
        targetDeps_dir.."/carbonite/include",        
        kit_sdk_includes,
        targetDeps_dir.."/"..physxVersion.."/pvdruntime/include",
        targetDeps_dir.."/"..physxVersion.."/include",
        runtime_include_dir
    }
    links { "physxSchema", "omni.usd", "foundation" }
    filter { "configurations:debug" }
        runtime "Debug"
    filter  { "configurations:release" }
        runtime "Release"
        if omniPhysXPvdOptimize then
            optimize "Speed"
        else
            optimize "off"
            runtime "Release"
            print("omni.physx.pvd: Disabling optimization for release.")            
        end    
    filter { "system:windows" }
        libdirs { targetDeps_dir.."/assimp/Lib/x64", }
    filter { "system:windows", "platforms:x86_64", "configurations:debug" }
        repo_build.copy_to_targetdir(root.."/_build/target-deps/"..physxVersion..'/bin/win.x86_64.vc142.md/debug/PVDRuntime_64.dll')
    filter { "system:windows", "platforms:x86_64", "configurations:release" }    
        repo_build.copy_to_targetdir(root.."/_build/target-deps/"..physxVersion..'/bin/win.x86_64.vc142.md/'..physxLibs..'/PVDRuntime_64.dll')
    filter { "system:linux" }
        exceptionhandling "On"
        removeflags { "FatalCompileWarnings", "UndefinedIdentifiers" }
        links { "rt" }
    filter { "system:linux", "platforms:x86_64", "configurations:debug" }
        repo_build.copy_to_targetdir(root.."/_build/target-deps/"..physxVersion..'/bin/linux.x86_64/debug/libPVDRuntime_64.so')
    filter { "system:linux", "platforms:x86_64", "configurations:release" }
        repo_build.copy_to_targetdir(root.."/_build/target-deps/"..physxVersion..'/bin/linux.x86_64/'..physxLibs..'/libPVDRuntime_64.so')          
    filter {}
    files { "python/**.py" }
    vpaths { ['python/*'] = "python/**.py" }

project ("omni.physx.pvd.python")
    carboniteBindingsPython {
        name = "_physxPvd",
        folder = "bindings",
        namespace = "omni" 
    }
    targetdir (targetDir.."/"..ext_dir.."/omni/physxpvd/bindings")
    includedirs {
        runtime_include_dir,
        targetDeps_dir.."/carbonite/include",
        targetDeps_dir.."/rtx_plugins/include",
        kit_sdk_includes,
        
    }
    
