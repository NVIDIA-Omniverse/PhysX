local ext_name = "omni.physics.physx.ui"
local ext_dir = extsPhysics_repo_dir.."/"..ext_name

define_physics_test_experience(ext_name)
define_etm_test_experience(ext_name)

group ("simulation")

repo_build.prebuild_link 
{
  { "config", ext_dir.."/config" },
  { "data/images", ext_dir.."/data/images" },
  { "docs", ext_dir.."/docs" },
  { "python/", ext_dir.."/omni/physics/physxui/" },
}
