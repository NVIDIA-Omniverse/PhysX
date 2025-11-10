local ext_name = "omni.physx.asset_validator"
local ext_dir = extsPhysics_dir.."/"..ext_name

define_physics_test_experience(ext_name, {dir = "extsPhysicsRepo"})
define_etm_test_experience(ext_name)

group ("simulation")

repo_build.prebuild_copy {    
    { "python/__init__.py", ext_dir.."/omni/physxassetvalidator/__init__.py" },    
}

repo_build.prebuild_link {
    { "config", ext_dir.."/config" },
    { "docs", ext_dir.."/docs" },
    { "data", ext_dir.."/data" },
    { "python/scripts", ext_dir.."/omni/physxassetvalidator/scripts" },
    { "python/tests", ext_dir.."/omni/physxassetvalidator/tests" },
    { repo_root_dir.."/data/usd/tests/Physics/AssetValidator", ext_dir.."/testdata" },
}

project ("omni.physx.asset_validator")
    carbonitePlugin { ifaces = "%{root}/include/omni/physx", impl = "plugins", targetname = "omni.physx.asset_validator.plugin" }
    staticruntime "Off"
    rtti "On"
    extension_usd_deps(targetDeps_dir, hostDeps_dir)
    extension_physxsdk_deps(targetDeps_dir, physxVersion)
    targetdir (targetDir.."/"..ext_dir.."/bin/")
    dependson { "prebuild" }
    includedirs {
        runtime_include_dir,
        targetDeps_dir.."/carbonite/include",
        schema_package_dir.."/include",
        kit_sdk_dir.."/dev/fabric/include",
        targetDeps_dir.."/client-library/include", -- UsdUtils.h
        kit_sdk_includes,-- UsdUtils.h
    }
    
    libdirs { 
        schema_package_dir.."/lib",
    }
    filter  { "configurations:release" }
    if omniPhysXAssetValidatorOptimize then
        optimize "Speed"
    else
        optimize "off"
        print("omni.physx.asset_validator.plugin: Disabling optimization for release.")            
        runtime "Release"
    end       
    filter {}
    link_boost_for_windows_wdefault()
    filter { "system:linux" }
        exceptionhandling "On"
    filter {}
    links { "carb", "physxSchema" }
    files { "python/**.py" }
    vpaths { ['python/*'] = "python/**.py" }

project ("omni.physx.asset_validator.python")
    carboniteBindingsPython {
        name = "_physxAssetValidator",
        folder = "bindings",
        namespace = "omni" }    
    includedirs {
        runtime_include_dir,
        schema_package_dir.."/include",       
        kit_sdk_includes,
    }    
    targetdir (targetDir.."/"..ext_dir.."/omni/physxassetvalidator/bindings")
    filter  { "configurations:release" }
    if omniPhysXAssetValidatorOptimize then
        optimize "Speed"
    else
        optimize "off"
        print("omni.physx.asset_validator.python: Disabling optimization for release.")            
        runtime "Release"
    end       
    filter {}
