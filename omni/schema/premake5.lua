include("../ovexts/premake5-public.lua")

repo_build = require("omni/repo/build")
repo_build.setup_options()

root = repo_build.get_abs_path(".")
target_deps = root.."/_build/target-deps"
host_deps = root.."/_build/host-deps"
schema_source_dir = root.."/source"

repo_build.set_prebuild_file('_build/generated/prebuild.toml')

-- Starting from here we define a structure of actual solution to be generated. Starting with solution name.
workspace "physx-schemas"
    startproject ""

    repo_build.setup_workspace {
        msvc_version = MSVC_VERSION,
        winsdk_version = WINSDK_VERSION,
        extra_warnings = false,
        security_hardening = true,
        host_deps_dir = host_deps,
        target_deps_dir = target_deps,
        fix_cpp_version = true,
        use_stack_clash_protection = false, -- OVX gcc does not support this
    }

    filter { "system:windows" }
        platforms { "x86_64" }
        disablewarnings { "4244", "4305", "4996" }

    filter { "system:linux" }
        platforms { "x86_64", "aarch64" }
        defaultplatform "x86_64"
        buildoptions { "-D_FILE_OFFSET_BITS=64" }
        -- Add library origin directory to dlopen() search path
        linkoptions { "-Wl,-rpath,'$$ORIGIN' -Wl,--export-dynamic" }
        enablewarnings { "all" }

    filter { "system:linux" }
        disablewarnings{ "error=format", "error=format-security" }

    filter { "platforms:x86_64" }
        architecture "x86_64"
    filter {}

    group "schemas"
        include("premake5-local.lua")
