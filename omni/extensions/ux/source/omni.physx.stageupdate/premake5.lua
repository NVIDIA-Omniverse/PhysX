local ext_name = "omni.physx.stageupdate"
local ext_dir = extsPhysics_dir.."/"..ext_name

define_physics_test_experience(ext_name)
define_etm_test_experience(ext_name)

group ("simulation")

-- copy the python init file, this does initialize the module
repo_build.prebuild_copy {    
    { "python/__init__.py", ext_dir.."/omni/physxstageupdate/__init__.py" },    
}

repo_build.prebuild_link {
    { "config", ext_dir.."/config" },
    { "data", ext_dir.."/data" },
    { "docs", ext_dir.."/docs" },
    { "python/scripts", ext_dir.."/omni/physxstageupdate/scripts" },
    { "python/tests", ext_dir.."/omni/physxstageupdate/tests" },
}

project ("omni.physx.stageupdate")
    --iface example, source is expected in plugins directory
    carbonitePlugin { ifaces = "%{root}/include/omni/physx", impl = "plugins", targetname = "omni.physx.stageupdate.plugin" }
    staticruntime "Off"
    rtti "On"
    targetdir (targetDir.."/"..ext_dir.."/bin/")
    dependson { "prebuild", "omni.physx" }
    includedirs {
        runtime_include_dir,
        targetDeps_dir.."/carbonite/include",
        kit_sdk_includes,
    }

    filter { "system:linux" }
        links { "dl" }
    filter {}
    
    files { "python/**.py" }
    vpaths { ['python/*'] = "python/**.py" }

project ("omni.physx.stageupdate.python")
    carboniteBindingsPython {
        name = "_physxStageUpdateNode",
        folder = "bindings",
        namespace = "omni" 
    }
    targetdir (targetDir.."/"..ext_dir.."/omni/physxstageupdate/bindings")
    includedirs {
        runtime_include_dir,
    }
