local ext_name = "omni.usdphysics.tests"
local ext_dir = extsPhysics_dir.."/"..ext_name

define_physics_test_experience(ext_name)
define_etm_test_experience(ext_name)

group ("simulation")

repo_build.prebuild_copy {    
    { "python/__init__.py", ext_dir.."/omni/usdphysicstests/__init__.py" },    
}

repo_build.prebuild_link {
    { "config", ext_dir.."/config" },
    { "data", ext_dir.."/data" },
    { "docs", ext_dir.."/docs" },
    { "python/scripts", ext_dir.."/omni/usdphysicstests/scripts" }, 
}

project ("omni.usdphysics.tests.plugin")
    carbonitePlugin { ifaces = "%{root}/include/omni/physics/schematests", impl = "plugins", targetname = "omni.usdphysics.tests.plugin" }
    staticruntime "Off"
    rtti "On"
    extension_usd_deps(targetDeps_dir, hostDeps_dir)
    targetdir (targetDir.."/"..ext_dir.."/bin/")
    dependson { "prebuild", "omni.usdphysics.plugin" }
    includedirs {
        targetDeps_dir.."/carbonite/include",
    }    
    links { }
    filter { "system:linux" }
        exceptionhandling "On"
    filter {}
    files { "python/**.py" }
    vpaths { ['python/*'] = "python/**.py" }

project ("omni.usdphysics.tests.python")
    carboniteBindingsPython {
        name = "_usdPhysicsTests",
        folder = "bindings",
        namespace = "omni" 
    }
    extension_usd_deps(targetDeps_dir, hostDeps_dir)
    targetdir (targetDir.."/"..ext_dir.."/omni/usdphysicstests/bindings")
    link_boost_for_windows_wdefault()

