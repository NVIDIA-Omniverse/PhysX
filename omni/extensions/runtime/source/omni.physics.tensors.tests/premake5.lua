local ext_name = "omni.physics.tensors.tests"
local ext_dir = extsPhysics_dir.."/"..ext_name

define_physics_test_experience(ext_name, {suite="tensors"})

group ("simulation")

repo_build.prebuild_copy {    
    { "python/__init__.py", ext_dir.."/omni/physicstensorstests/__init__.py" },
}

repo_build.prebuild_link {
    { "config", ext_dir.."/config" },
    { "python/scripts", ext_dir.."/omni/physicstensorstests/scripts" },
    { "data", ext_dir.."/data" },
}
