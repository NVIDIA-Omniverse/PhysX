local ext_name = "omni.physx.tests.visual"
local ext_dir = extsPhysics_dir.."/"..ext_name

define_physics_test_experience(ext_name, {suite="visual"})
define_etm_test_experience(ext_name)

group ("simulation")

repo_build.prebuild_link 
{
  { "config", ext_dir.."/config" },
  { "docs", ext_dir.."/docs" },
  { "python/", ext_dir.."/omni/physxtestsvisual/" },
  { repo_root_dir.."/data/usd/tests/Physics/Visual/", ext_dir.."/data/" },
  { repo_root_dir.."/data/icons/omni.physx.demos", ext_dir.."/icons" },
}

project ("omni.physx.tests.visual")
    kind "Utility"
    files { "python/**.py" }
    vpaths { ['python/*'] = "python/**.py" }
    location (workspaceDir.."/%{prj.name}")
