
-- copy files into exts folder
repo_build.prebuild_copy
{
    { "generatedSchema.usda",  root.."/_build/%{cfg.system}-%{cfg.platform}/%{cfg.buildcfg}/schema/share/usd/plugins/PhysxSchemaAddition/resources" },
    { "schema.usda",  root.."/_build/%{cfg.system}-%{cfg.platform}/%{cfg.buildcfg}/schema/share/usd/plugins/PhysxSchemaAddition/resources" },
    { "plugInfo.json",  root.."/_build/%{cfg.system}-%{cfg.platform}/%{cfg.buildcfg}/schema/share/usd/plugins/PhysxSchemaAddition/resources" },
}
