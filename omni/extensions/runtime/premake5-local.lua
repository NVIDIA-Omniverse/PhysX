-- physics runtime
group "runtime"
    dofile ("source/omni.convexdecomposition/premake5.lua")
    dofile ("source/omni.physx/premake5.lua")
    dofile ("source/omni.physx.foundation/premake5.lua")
    dofile ("source/omni.physx.fabric/premake5.lua")
    dofile ("source/omni.usdphysics/premake5.lua")
    dofile ("source/omni.usdphysics.tests/premake5.lua")

    -- tensor extensions can be excluded to get around CUDA build bug
    if (not _OPTIONS["notensors"]) then
        dofile ("source/omni.physics.tensors/premake5.lua")
        dofile ("source/omni.physics.tensors.tests/premake5.lua")
        dofile ("source/omni.physx.tensors/premake5.lua")
    end

    -- schema extensions
    dofile ("source/omni.usd.schema.physx/premake5.lua")

group "tests"
    local targetPluginsDir = targetDir.."/plugins"
    local winTargetDir = root.."/_build/windows-x86_64/"..config

    project "test.unit"
        kind "ConsoleApp"
        dependson { "prebuild", "omni.physx.plugin" }
        includedirs {
            "include",
            kit_sdk_includes,
            repo_root_dir.."/include/extras",
            targetDeps_dir.."/rtx_plugins/include",
            schema_package_dir.."/include",
            kit_sdk_dir.."/dev/fabric/include",
            targetDeps_dir.."/usdrt/include",
            targetDeps_dir.."/gsl/include",   -- Support for std::span
            targetDeps_dir.."/doctest/include",
            "../common/include",
        }

        targetdir (targetPluginsDir)
                -- Turn on some options for doctest
        -- See documentation here: https://github.com/onqtam/doctest/blob/master/doc/markdown/configuration.md#doctest_config_disable
        defines { "DOCTEST_CONFIG_TREAT_CHAR_STAR_AS_STRING", "DOCTEST_CONFIG_SUPER_FAST_ASSERTS" }
        defines { "TBB_SUPPRESS_DEPRECATED_MESSAGES",}
        location (workspace_dir.."/%{prj.name}")
        language "C++"
        staticruntime "Off"
        exceptionhandling "On"
        extension_physxsdk_deps(targetDeps_dir, physxVersion)
        extension_usd_deps_tests(targetDeps_dir, hostDeps_dir)
        inlining "Explicit"
        filter { "configurations:debug" }
            runtime "Debug"
        filter  { "configurations:release" }
            runtime "Release"
            if testUnitOptimize then
                optimize "Speed"
            else
                optimize "off"
                print("Unit tests: Disabling optimization for release.")
            end

        filter {}
		links { "cuda", "carb" }
        add_cuda_dirs()
        files {"tests/test.unit/**.h", "tests/test.unit/**.cpp", "tests/common/**.h", "tests/common/**.cpp"}
        vpaths { ['/*'] = "tests/test.unit/**.*" }
        libdirs {
            schema_package_dir.."/lib",
        }

        filter { "system:windows" }
            libdirs { targetDeps_dir.."/python/libs" }
            links {  "physicsSchemaTools", "physxSchema" }
            debugenvs { "PATH=%%PATH%%;"..winTargetDir.."/kit;"..winTargetDir.."/kit/kernel/plugins;"..winTargetDir.."/exts/omni.usd.libs/bin;"..winTargetDir.."/extsPhysics/omni.usd.schema.physics/bin;"..winTargetDir.."/extsPhysics/omni.usd.schema.physx/bin" }
        filter { "system:linux" }
            buildoptions { "-pthread" }
            buildoptions { "-fPIC" }
            links { "physicsSchemaTools", "physxSchema", BOOST_LIB, PYTHON_LIB, "dl", "pthread", "rt" }
            libdirs {
                targetDeps_dir.."/cuda/lib64",
                targetDeps_dir.."/cuda/lib64/stubs",
                kit_sdk_dir.."/python/lib",
            }
            disablewarnings { "error=switch", "error=sign-compare" }
        filter { "system:linux", "configurations:debug" }
            links { "tbb_debug" }
        filter { "system:linux", "configurations:release" }
            links { "tbb" }
        filter {}

if os.target() == "windows" then
    usd_env_var = "%~dp0kit;%~dp0kit/kernel/plugins;%~dp0exts/omni.usd.libs/bin;%~dp0extsPhysics/omni.usd.schema.physx/bin;%~dp0extsPhysics/omni.usd.schema.physics/bin"
else
    usd_env_var = "$SCRIPT_DIR/kit:$SCRIPT_DIR/kit/plugins/carb_gfx:$SCRIPT_DIR/kit/python/lib:$SCRIPT_DIR/exts/omni.usd.libs/bin:$SCRIPT_DIR/extsPhysics/omni.usd.schema.physx/bin:$SCRIPT_DIR/extsPhysics/omni.usd.schema.physics/bin"
end

create_test_runner("test.unit", "debug", usd_env_var)
create_test_runner("test.unit", "release", usd_env_var)
