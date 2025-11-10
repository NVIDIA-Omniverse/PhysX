local ext_name = "omni.physx.graph"
local ext_dir = extsPhysics_dir.."/"..ext_name
local ext_dir_subpath = "omni/physxgraph"
local ext_dir_with_subpath = ext_dir.."/"..ext_dir_subpath

define_physics_test_experience(ext_name)
define_etm_test_experience(ext_name)

local ext_info = get_current_extension_info()
-- adjust values as the automatic extraction does not really fit in our case
ext_info["bin_dir"] = ext_dir.."/bin"
ext_info["target_dir"] = ext_dir
ext_info["group"] = "simulation"

local ogn_info = get_ogn_project_information(ext_info, ext_dir_subpath)
-- adjust values as the automatic extraction does not really fit in our case
ogn_info["bindings_module"] = "_physxGraph"
ogn_info["plugin_project"] = "omni.physx.graph.plugin"
ogn_info["ogn_project"] = "omni.physx.graph.ogn"
ogn_info["python_project"] = "omni.physx.graph.python"
ogn_info["dependson"][5] = ogn_info["ogn_project"]  --!!! this is not future proof (see OM-47264)
ogn_info["docs_target_path"] = "%{root}/_build/ogn/docs/omni.physx.graph"

group ("simulation")

repo_build.prebuild_copy {    
    { "python/__init__.py", ext_dir.."/omni/physxgraph/__init__.py" }
}

repo_build.prebuild_link {
    { "docs", ext_dir.."/docs" },
    { "data", ext_dir.."/data" },
    { "config", ext_dir.."/config" },
    { "nodes", ext_dir.."/omni/physxgraph/ogn/nodes" },
    { "python/scripts", ext_dir.."/omni/physxgraph/scripts" },
    { "python/tests", ext_dir.."/omni/physxgraph/tests" },
}

project ("omni.physx.graph.plugin")
    carbonitePlugin { ifaces = "%{root}/include/omni/physx", impl = "plugins", targetname = "omni.physx.graph.plugin" }
    staticruntime "Off"
    rtti "On"
    extension_usd_deps(targetDeps_dir, hostDeps_dir)
    extension_physxsdk_deps(targetDeps_dir, physxVersion)
    targetdir (targetDir.."/"..ext_dir.."/bin")
    dependson { "prebuild", "carb.physics-usd.plugin", "omni.physx.plugin", "foundation" }
    -- Add the standard dependencies all OGN projects have
    add_ogn_dependencies(ogn_info)
    removelinks {"ar","gf","arch","plug","sdf","tf","usd","vt","usdUtils"}
    
    install_ogn_configuration_file("plugins/categories.json")
    
    includedirs {
        targetDeps_dir.."/carbonite/include",
        targetDeps_dir.."/gsl/include",
        kit_sdk_includes,
        runtime_include_dir
    }
    links { "physxSchema", "omni.usd", "carb", "foundation", "physicsSchemaTools" }
    filter { "system:windows" }
        libdirs { targetDeps_dir.."/assimp/Lib/x64", }
    filter { "system:linux" }
        exceptionhandling "On"
    filter  { "configurations:release" }
        runtime "Release"
        if omniPhysXGraphOptimize then
            optimize "Speed"
        else
            optimize "off"
            runtime "Release"
            print("omni.physx.graph: Disabling optimization for release.")            
        end       
    filter {}
    files { "python/**.py" }
    vpaths { ['python/*'] = "python/**.py" }
    files { "nodes/**.cpp", "nodes/**.ogn" }
    vpaths { ['nodes/*'] = { "nodes/**.cpp", "nodes/**.ogn" } }
    
project ("omni.physx.graph.python")
    carboniteBindingsPython {
        name = "_physxGraph",
        folder = "bindings",
        namespace = "omni" } 

    extension_usd_deps(targetDeps_dir, hostDeps_dir)
    includedirs { runtime_include_dir }        
    targetdir (targetDir.."/"..ext_dir.."/omni/physxgraph/bindings")
    link_boost_for_windows_wdefault()


-- To avoid that a sub group named "omni.physx.graph" is created below (see OM-47264)
ext_info["id"] = ""

-- Breaking this out as a separate project ensures the .ogn files are processed before their results are needed
project_ext_ogn(ext_info, ogn_info)
