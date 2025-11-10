local ext_name = "omni.physx.demos"
local ext_dir = extsPhysics_dir.."/"..ext_name

-- main suite except any demos with external assets
define_physics_test_experience("omni.physx.demos",
  {
    dir="extsPhysicsRepo",
    suite="demos",
    test_args={
      "--", -- push the args below as unprocessed args through exttest processing
      "--/exts/omni.kit.test/excludeTests/0='*PhysxXAssetDemosTest*'"
    }
  }
)

-- demos with external assets, fix path to s3
define_physics_test_experience("omni.physx.demos", 
  {
    dir="extsPhysicsRepo",
    suite="assetdemos",
    suffix=".s3",
    test_args={
      "--", -- push the args below as unprocessed args through exttest processing
      "--/physics/demoDevelopmentMode=0",
      "--/exts/omni.kit.test/runTestsFilter='*PhysxXAssetDemosTest*'"
    }
  }
)

define_etm_test_experience(ext_name)

group ("simulation")

repo_build.prebuild_link 
{
  { "config", ext_dir.."/config" },
  { repo_root_dir.."/data/icons/omni.physx.demos", ext_dir.."/icons" },
  { "docs", ext_dir.."/docs" },
  { "python/", ext_dir.."/omni/physxdemos/" },
  { repo_root_dir.."/data/usd/tests/Physics/Demos", ext_dir.."/data" },
  { repo_root_dir.."/data/audio/physics/demo", ext_dir.."/audio" },
}

project ("omni.physx.demos.plugin")
    kind "Utility"
    files { "python/**.py" }
    vpaths { ['python/*'] = "python/**.py" }
    targetdir (targetDir.."/"..ext_dir.."/omni/physxdemos")
    location (workspaceDir.."/%{prj.name}")
