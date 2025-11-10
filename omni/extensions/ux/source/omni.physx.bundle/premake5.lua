local ext_name = "omni.physx.bundle"
local ext_dir = extsPhysics_dir.."/"..ext_name

define_physics_test_experience(ext_name)
define_etm_test_experience(ext_name)

repo_build.prebuild_link 
{
    { "config", ext_dir.."/config" },
	{  "docs", ext_dir.."/docs"  },
    {  "data", ext_dir.."/data"  },
}