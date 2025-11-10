local ext_name = "omni.physx.ui"
local ext_dir = extsPhysics_dir.."/"..ext_name

define_physics_test_experience(ext_name)
define_etm_test_experience(ext_name)

group ("simulation")

repo_build.prebuild_copy {
    { "python/__init__.py", ext_dir.."/omni/physxui/__init__.py" }
}

repo_build.prebuild_link {
    { "config", ext_dir.."/config" },
    { "data", ext_dir.."/data" },
    { "docs", ext_dir.."/docs" },
    { "python/scripts", ext_dir.."/omni/physxui/scripts" },
    { "python/tests", ext_dir.."/omni/physxui/tests" },    
    { repo_root_dir.."/data/icons/omni.physx.ui", ext_dir.."/icons" },
    { repo_root_dir.."/extensions/common/python/windowmenuitem", ext_dir.."/omni/physxuicommon/windowmenuitem" },
}

project ("omni.physxui.plugin")
    carbonitePlugin { ifaces = "%{root}/include/omni/physx", impl = "plugins", targetname = "omni.physxui.plugin" }
    staticruntime "Off"
    rtti "On"
    extension_usd_deps(targetDeps_dir, hostDeps_dir)
    extension_omniui_deps()
    extension_imgui_deps()
    targetdir (targetDir.."/"..ext_dir.."/bin/")
    dependson { "prebuild", "carb.physics-usd.plugin", "omni.physx.plugin", "foundation" }
    language "C++"
    includedirs {   
        targetDeps_dir.."/carbonite/include",
        targetDeps_dir.."/rtx_plugins/include",
        targetDeps_dir.."/gsl/include",   -- Support for std::span
        kit_sdk_dir.."/dev/fabric/include",
        targetDeps_dir.."/client-library/include",
        targetDeps_dir.."/glm",
        kit_sdk_includes,
        runtime_include_dir
    }
    libdirs {
        kit_sdk_dir.."/plugins",
    }
    links { "physicsSchemaTools", "physxSchema", "omni.usd", "foundation", "omni.ui", "omni.ui.scene", "carb" }

    filter { "configurations:debug" }
        runtime "Debug"
    filter  { "configurations:release" }
        runtime "Release"
        if omniPhysXUiOptimize then
            optimize "Speed"
        else
            optimize "off"
            runtime "Release"
            print("omni.physx.ui: Disabling optimization for release.")
        end
    filter { "system:linux" }
        exceptionhandling "On"
        links { "rt" }
    filter { "system:linux", "platforms:x86_64", "configurations:debug" }
        buildoptions { "-Wno-invalid-offsetof" }
    filter { "system:linux", "platforms:x86_64", "configurations:release" }
        buildoptions { "-Wno-invalid-offsetof" }
    filter {}
    files { "python/**.py" }
    vpaths { ['python/*'] = "python/**.py" }


project ("omni.physxui.python")
    carboniteBindingsPython {
        name = "_physxUI",
        folder = "bindings",
        namespace = "omni" }
    extension_usd_deps(targetDeps_dir, hostDeps_dir)
    extension_omniui_deps()
    includedirs {
        kit_sdk_includes,
        runtime_include_dir
    }
    targetdir (targetDir.."/"..ext_dir.."/omni/physxui/bindings")
    links { "omni.ui.scene" }
    link_boost_for_windows_wdefault()

