local ext_name = "omni.physx.cct"
local ext_dir = extsPhysics_dir.."/"..ext_name
local ext_dir_subpath = "omni/physxcct"

define_physics_test_experience(ext_name)
define_etm_test_experience(ext_name)

local projGroup = "simulation"

local ext_info = get_current_extension_info()
ext_info["bin_dir"] = ext_dir.."/bin"
ext_info["target_dir"] = ext_dir
ext_info["group"] = projGroup

local ogn_info = get_ogn_project_information(ext_info, ext_dir_subpath)
ogn_info["bindings_module"] = "_physxCct"
ogn_info["plugin_project"] = "omni.physx.cct.plugin"
ogn_info["ogn_project"] = "omni.physx.cct.ogn"
ogn_info["python_project"] = "omni.physx.cct.python"
ogn_info["dependson"][5] = ogn_info["ogn_project"]
ogn_info["docs_target_path"] = "%{root}/_build/ogn/docs/omni.physx.cct"

group (projGroup)

repo_build.prebuild_copy {    
    { "python/__init__.py", ext_dir.."/omni/physxcct/__init__.py" },    
}

repo_build.prebuild_link {
    { "config", ext_dir.."/config" },
    { "data", ext_dir.."/data" },
    { "docs", ext_dir.."/docs" },
    { "python/scripts", ext_dir.."/omni/physxcct/scripts" },
    { "nodes", ext_dir.."/omni/physxcct/ogn/nodes" },
}

project ("omni.physx.cct.plugin")
    carbonitePlugin { ifaces = "%{root}/include/omni/physx", impl = "plugins", targetname = "omni.physx.cct.plugin" }
    staticruntime "Off"
    rtti "On"
    extension_usd_deps(targetDeps_dir, hostDeps_dir)
    targetdir (targetDir.."/"..ext_dir.."/bin")
    dependson { "prebuild", "carb.physics-usd.plugin", "omni.physx.plugin" }
    add_ogn_dependencies(ogn_info)
    removelinks {"ar","gf","arch","plug","sdf","tf","usd","vt","usdUtils"}
    includedirs {
        runtime_include_dir,
        targetDeps_dir.."/carbonite/include",
        targetDeps_dir.."/"..physxVersion.."/include",
        targetDeps_dir.."/rtx_plugins/include",
        kit_sdk_includes,
    }
    filter { "system:windows" }
        libdirs { targetDeps_dir.."/assimp/Lib/x64", }
        links { "physxSchema", "omni.usd" }
    filter { "configurations:debug" }
        runtime "Debug"
    filter  { "configurations:release" }
        runtime "Release"
        if omniPhysXCctOptimize then
            optimize "Speed"
        else
            optimize "off"
            runtime "Release"
            print("omni.physx.cct: Disabling optimization for release.")            
        end
    filter { "system:windows", "platforms:x86_64", "configurations:debug" }
        libdirs { targetDeps_dir.."/"..physxVersion.."/bin/win.x86_64.vc142.md/debug" }
        defines {  "PX_PHYSX_STATIC_LIB", "_DEBUG" }
    filter { "system:windows", "platforms:x86_64", "configurations:release" }
        libdirs { targetDeps_dir.."/"..physxVersion.."/bin/win.x86_64.vc142.md/"..physxLibs}
        defines {  "PX_PHYSX_STATIC_LIB", "NDEBUG" }
    filter { "system:windows" }
        links { "PhysXCharacterKinematic_static_64" }
    filter { "system:linux" }
        exceptionhandling "On"
        links { "PhysXCharacterKinematic_static_64" }          
        links { "physxSchema", "omni.usd" }
    filter { "system:linux", "platforms:x86_64", "configurations:debug" }
        buildoptions { "-Wno-invalid-offsetof" }
        defines {  "PX_PHYSX_STATIC_LIB", "_DEBUG" }
        libdirs { targetDeps_dir.."/"..physxVersion.."/bin/linux.x86_64/debug" }
    filter { "system:linux", "platforms:aarch64", "configurations:debug" }     
        buildoptions { "-Wno-invalid-offsetof" }    
        defines {  "PX_PHYSX_STATIC_LIB", "_DEBUG" }    
        libdirs { targetDeps_dir.."/"..physxVersion.."/bin/linux.aarch64/debug" }   
    filter { "system:linux", "platforms:x86_64", "configurations:release" }
        buildoptions { "-Wno-invalid-offsetof" }
        defines {  "PX_PHYSX_STATIC_LIB", "NDEBUG" }
        libdirs { targetDeps_dir.."/"..physxVersion.."/bin/linux.x86_64/"..physxLibs }
    filter { "system:linux", "platforms:aarch64", "configurations:release" }
        buildoptions { "-Wno-invalid-offsetof" }
        defines {  "PX_PHYSX_STATIC_LIB", "NDEBUG" }
        libdirs { targetDeps_dir.."/"..physxVersion.."/bin/linux.aarch64/"..physxLibs }        
    filter {}
    files { "python/**.py" }
    vpaths { ['python/*'] = "python/**.py" }

project ("omni.physx.cct.python")
    carboniteBindingsPython {
        name = "_physxCct",
        folder = "bindings",
        namespace = "omni" 
    }
    targetdir (targetDir.."/"..ext_dir.."/omni/physxcct/bindings")
    includedirs { runtime_include_dir }

project_ext_ogn(ext_info, ogn_info)
