local ext_name = "omni.physx.commands"
local ext_dir = extsPhysics_dir.."/"..ext_name

define_physics_test_experience(ext_name)
define_etm_test_experience(ext_name)

group ("simulation")

repo_build.prebuild_link 
{
  { "config", ext_dir.."/config" },
  { "data", ext_dir.."/data" },
  { "docs", ext_dir.."/docs" },
  { "python/", ext_dir.."/omni/physxcommands/" },
}

project ("omni.physx.commands.plugin")
    kind "Utility"
    files { "python/**.py" }
    vpaths { ['python/*'] = "python/**.py" }
    location (workspaceDir.."/%{prj.name}")
