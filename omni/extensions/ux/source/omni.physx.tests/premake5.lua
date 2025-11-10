local ext_name = "omni.physx.tests"
local ext_dir = extsPhysics_dir.."/"..ext_name

define_physics_test_experience(ext_name, {suite="omniphysx"})
define_etm_test_experience(ext_name)

group ("simulation")

repo_build.prebuild_link 
{
  { "config", ext_dir.."/config" },
  { "docs", ext_dir.."/docs" },
  { "python/", ext_dir.."/omni/physxtests/" },
  { repo_root_dir.."/data/icons/omni.physx.demos", ext_dir.."/icons" },
  { repo_root_dir.."/data/usd/tests/Physics/Base/", ext_dir.."/data/" },
}

project ("omni.physx.tests.plugin")
    kind "Utility"
    files { "python/**.py" }
    vpaths { ['python/*'] = "python/**.py" }
    location (workspaceDir.."/%{prj.name}")