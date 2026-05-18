local ext_name = "omni.kit.property.physics"
local ext_version = ""
local ext_id = ext_name

define_physics_test_experience(ext_name)
define_etm_test_experience(ext_name)

group ("simulation")

repo_build.prebuild_link 
{
    { "config", extsPhysics_dir.."/omni.kit.property.physics/config" },
    { "data", extsPhysics_dir.."/omni.kit.property.physics/data" },
    { "docs", extsPhysics_dir.."/omni.kit.property.physics/docs" },
    { "python/", extsPhysics_dir.."/omni.kit.property.physics/omni/kit/property/physics/" },
}

project ("omni.kit.property.physics.plugin")
    kind "Utility"
    files { "python/**.py" }
    vpaths { ['python/*'] = "python/**.py" }
    location (workspaceDir.."/%{prj.name}")
