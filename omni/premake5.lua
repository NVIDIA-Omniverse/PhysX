include("premake5-public.lua")
define_common("", "/extensions")

workspace "Physics"
    generate_version_header()
    workspace_defaults(currentAbsPath, hostDepsDir, targetDepsDir)
    add_generate_solution_project()

-- FIXME: disabled to build on newer gccs on gitlab
filter { "system:linux" }
   disablewarnings { "class-memaccess", "range-loop-construct", "maybe-uninitialized", "reorder", "strict-aliasing", "switch", "sign-compare", "narrowing", "attributes", "enum-compare", "format", "uninitialized", "pointer-arith", "nonnull-compare", "parentheses", "delete-incomplete", "unused-variable", "unused-but-set-variable", "unused-function", "changes-meaning", "error=overloaded-virtual" }
    
filter {}

filter { "system:linux" }
    disablewarnings{ "error=format", "error=format-security" }
filter {}

if _OPTIONS["devschema"] then
    dofile("schema/premake5-local.lua")
    local shell_ext=".sh"
    if os.target() == "windows" then
        shell_ext=".bat"
    end    
    postbuildcommands {root.."/tools/packman/python"..shell_ext.." "..root.."/schema/tools/gen_unit_database.py "..local_schema_dir.."/lib/python/PhysicsSchemaTools"}
end

group "common"
    dofile ("extensions/common/premake5.lua")

include("extensions/runtime/premake5-local.lua")
include("extensions/ux/premake5-local.lua")

filter { "system:windows" }
    disablewarnings { "4244", "4305", "4996" }
filter {}

if os.target() == "windows" and _OPTIONS["devphysx"] then
    include_physxsdk()
end

-- python apps
repo_build.prebuild_link {
    { "extensions/tests/pythonapps/apps", bin_dir.."/pythonapps" },
}
repo_build.prebuild_copy {
    { "extensions/tests/pythonapps/runscripts/*$shell_ext", bin_dir },
}

define_app("omni.bloky.physx.kit", { extra_args = "--portable --/renderer/debug/validation/enabled=false", define_test = false })

repo_build.prebuild_link {{ "apps", bin_dir.."/apps" }}
