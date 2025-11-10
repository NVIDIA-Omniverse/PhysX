local ext_name = "omni.usd.schema.physx"
local ext_dir = extsPhysics_dir.."/"..ext_name

define_physics_test_experience(ext_name)
define_etm_test_experience(ext_name)

group ("schema")

repo_build.prebuild_link 
{
    { "config", ext_dir.."/config" },
	{ "docs", ext_dir.."/docs"  },
    { "data", ext_dir.."/data"  },
    { "python/tests", ext_dir.."/usd/schema/physx/tests" },
    { "PACKAGE-LICENSES", ext_dir.."/PACKAGE-LICENSES" },
}

repo_build.prebuild_copy 
{
    { "python/__init__.py", ext_dir.."/usd/schema/physx" },
}

project(ext_name)
    repo_build.utility_project()
    location (workspaceDir.."/%{prj.name}")
    dependson { "physxSchema", "_physxSchema", "physicsSchemaTools", "_physicsSchemaTools" }
    repo_build.copy_to_dir(schema_package_dir.."/share/usd/plugins/PhysxSchema/resources/**", ext_dir.."/plugins/PhysxSchema/resources" )
    repo_build.copy_to_dir(schema_package_dir.."/lib/python/PhysicsSchemaTools/**", ext_dir.."/pxr/PhysicsSchemaTools" )
    repo_build.copy_to_dir(schema_package_dir.."/lib/python/PhysxSchema/**", ext_dir.."/pxr/PhysxSchema" )
    repo_build.copy_to_dir(schema_package_dir.."/share/usd/plugins/PhysxSchemaAddition/resources/**", ext_dir.."/plugins/PhysxSchemaAddition/resources" )
    repo_build.copy_to_dir(schema_package_dir.."/share/usd/plugins/OmniUsdPhysicsDeformableSchema/resources/**", ext_dir.."/plugins/OmniUsdPhysicsDeformableSchema/resources" )

    if os.target() == "windows" then
        repo_build.copy_to_dir(schema_package_dir.."/lib/physxSchema.dll", ext_dir.."/bin" )
        repo_build.copy_to_dir(schema_package_dir.."/lib/physicsSchemaTools.dll", ext_dir.."/bin" )
    end

    if os.target() == "linux" then
        repo_build.copy_to_dir(schema_package_dir.."/lib/libphysxSchema.so", ext_dir.."/bin" )
        repo_build.copy_to_dir(schema_package_dir.."/lib/libphysicsSchemaTools.so", ext_dir.."/bin" )
    end
