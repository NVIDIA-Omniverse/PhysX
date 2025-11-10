local ext_name = "omni.kit.property.physx"
local ext_version = ""
local ext_id = ext_name

define_physics_test_experience(ext_name)
define_etm_test_experience(ext_name)

group ("simulation")

repo_build.prebuild_link 
{
    { "config", extsPhysics_dir.."/omni.kit.property.physx/config" },
    { "data", extsPhysics_dir.."/omni.kit.property.physx/data" },
    { "docs", extsPhysics_dir.."/omni.kit.property.physx/docs" },
    { "python/", extsPhysics_dir.."/omni.kit.property.physx/omni/kit/property/physx/" },
}

project ("omni.kit.property.physx.plugin")
    kind "Utility"
    files { "python/**.py" }
    vpaths { ['python/*'] = "python/**.py" }
    location (workspaceDir.."/%{prj.name}")
