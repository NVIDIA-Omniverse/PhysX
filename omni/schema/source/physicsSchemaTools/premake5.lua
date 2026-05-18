-- Copyright 2023 NVIDIA CORPORATION
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--    http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

-- require base files
local usd_plugin = require(root.."/_repo/deps/repo_usd/templates/premake/premake5-usdplugin")

-- setup options for included methods
local options = {
    usd_root = root.."/_build/target-deps/usd/%{cfg.buildcfg}",
    usd_lib_prefix = "usd_",
    usd_suppress_warnings = true,
    python_root = root.."/_build/target-deps/python",
    plugin_include_dir = root.."/_build/%{cfg.system}-%{cfg.platform}/%{cfg.buildcfg}/schema/include/physicsSchemaTools",
    plugin_lib_dir = root.."/_build/%{cfg.system}-%{cfg.platform}/%{cfg.buildcfg}/schema/lib",
    plugin_module_dir = root.."/_build/%{cfg.system}-%{cfg.platform}/%{cfg.buildcfg}/schema/lib/python/PhysicsSchemaTools",
    plugin_resources_dir = root.."/_build/%{cfg.system}-%{cfg.platform}/%{cfg.buildcfg}/schema/share/usd/plugins/PhysicsSchemaTools/resources"
}

local usd_libs = {
    "arch","tf","vt","sdf","usd","usdGeom","gf","usdPhysics", "boost"
}

local public_headers = {
    "api.h","physicsSchemaTokens.h","UsdTools.h"
}

local private_headers = {
    
}

local cpp_files = {
    "moduleDeps.cpp","UsdTools.cpp","physicsSchemaTokens.cpp"
}

local python_module_cpp_files = {
    "module.cpp","wrapUsdTools.cpp"
}

local python_module_files = {
    "__init__.py"
}

local resource_files = {
}

repo_build.prebuild_copy {
    { schema_source_dir.."/physicsSchemaTools/units.py", repo_build.target_dir().."/schema/lib/python/PhysicsSchemaTools" },
}

-- USD plugin C++ project
project("physicsSchemaTools")
    -- standard USD plugin settings
    usd_plugin.usd_plugin("physicsSchemaTools", options, public_headers, private_headers, cpp_files, resource_files, usd_libs)
    usd_plugin.use_standard_usd_options()
    do_usd_zcinline_fix()
    link_boost_for_windows_wdefault()
    filter { "files:UsdTools.cpp", "system:linux"}
        buildoptions { "-Wno-strict-aliasing" }
    filter { "system:linux"}
        buildoptions { "-fvisibility=default" }
    filter { "system:linux", "configurations:debug" }
        libdirs { options.usd_root .. "/lib"}
        links { "tbb_debug" }

local count = 0
if python_module_cpp_files ~= nil then
    for _ in pairs(python_module_cpp_files) do
        count = count + 1
    end
end

if count > 0 then
    
project("_physicsSchemaTools")
    -- standard USD python plugin settings
    usd_plugin.usd_python_plugin("physicsSchemaTools", options, python_module_cpp_files, python_module_files, usd_libs)
    usd_plugin.use_standard_usd_options()
    do_usd_zcinline_fix()
    link_boost_for_windows_wdefault()
    filter { "system:linux", "configurations:debug" }
        libdirs { options.usd_root .. "/lib"}
        links { "tbb_debug" }
end
