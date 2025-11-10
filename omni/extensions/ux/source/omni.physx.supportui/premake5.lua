local ext_name = "omni.physx.supportui"
local ext_dir = extsPhysics_dir.."/"..ext_name

define_physics_test_experience(ext_name, {dir = "extsPhysicsRepo"})
define_etm_test_experience(ext_name)

group ("simulation")

repo_build.prebuild_copy {
    { "python/__init__.py", ext_dir.."/omni/physxsupportui/__init__.py" }
}

repo_build.prebuild_link {
    { "config", ext_dir.."/config" },
    { "docs", ext_dir.."/docs" },
    { "data", ext_dir.."/data" },
    { "python/scripts", ext_dir.."/omni/physxsupportui/scripts" },
    { "resources", ext_dir.."/resources" },
    { repo_root_dir.."/data/images/omni.physx.placement", ext_dir.."/icons" },
}

project ("omni.physx.supportui.plugin")
    carbonitePlugin { ifaces = "%{root}/include/omni/physx", impl = "plugins", targetname = "omni.physx.supportui.plugin" }
    staticruntime "Off"
    rtti "On"
    extension_usd_deps(targetDeps_dir, hostDeps_dir)
    extension_physxsdk_deps(targetDeps_dir, physxVersion)
    extension_omniui_deps()
    extension_imgui_deps()
    targetdir (targetDir.."/"..ext_dir.."/bin")
    dependson { "prebuild", "carb.physics-usd.plugin", "foundation" }
    includedirs {
        targetDeps_dir.."/carbonite/include",        
        kit_sdk_includes,
        targetDeps_dir.."/client-library/include",
        kit_sdk_dir.."/dev/fabric/include",
        targetDeps_dir.."/gsl/include",   -- Support for gsl::span
        "%{target_deps}/rtx_plugins/include",
        runtime_include_dir }
    filter { "configurations:debug" }
        runtime "Debug"
    filter  { "configurations:release" }
        runtime "Release"
        if omniPhysXSupportUiOptimize then
            optimize "Speed"
        else
            optimize "off"
            runtime "Release"
            print("omni.physx.supportui: Disabling optimization for release.")            
        end    
    filter { "system:linux" }
        disablewarnings { "error=deprecated-declarations" }
        exceptionhandling "On"
        links { "rt" }
    filter {}
    links { "physxSchema", "omni.usd", "carb", "foundation",  "omni.ui", "omni.ui.scene"}
    files { "python/**.py" }
    vpaths { ['python/*'] = "python/**.py" }

project ("omni.physx.supportui.python")
    carboniteBindingsPython {
        name = "_physxSupportUi",
        folder = "bindings",
        namespace = "omni" 
    }
    extension_usd_deps(targetDeps_dir, hostDeps_dir)
    extension_omniui_deps()
    includedirs {
        kit_sdk_includes,
        runtime_include_dir
    }    
    targetdir (targetDir.."/"..ext_dir.."/omni/physxsupportui/bindings")
    links { "omni.ui", "omni.ui.scene" }
    link_boost_for_windows_wdefault()

group ("common")
