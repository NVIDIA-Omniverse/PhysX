function usd_prefix(...)
    USD_PREFIX = "usd_"
    local libs = {...}
    prefixed_list = {}
    for _, lib in ipairs(libs[1]) do
        table.insert(prefixed_list, USD_PREFIX .. lib)
    end
    return prefixed_list
end

group "schema"
    dofile ("source/physxSchema/premake5.lua")
    dofile ("source/physxSchemaAddition/premake5.lua")
    dofile ("source/physicsSchemaTools/premake5.lua")
    dofile ("source/omniUsdPhysicsDeformableSchema/premake5.lua")

