-- Use folder name to build extension name and tag. Version is specified explicitly.
local ext_name = "omni.usd.metrics.assembler.physics"
local ext_dir = extsPhysics_dir.."/"..ext_name

define_physics_test_experience(ext_name)
define_etm_test_experience(ext_name)

group ("simulation")

repo_build.prebuild_link {
    { "config", ext_dir.."/config" },
    { "data", ext_dir.."/data" },
    { "docs", ext_dir.."/docs" },
    { "source/python", ext_dir.."/omni/metrics/assembler/physics" },
}