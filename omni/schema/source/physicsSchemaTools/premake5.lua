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
local usd_base = require(root.."/_repo/deps/repo_usd/templates/premake/premake5-usd")

-- setup options for included methods
local options = {
    usd_root = root.."/_build/target-deps/usd/%{cfg.buildcfg}",
    usd_suppress_warnings = true,
    -- python_root is omitted here so use_usd does not try to link the non-existent
    -- {prefix}python.lib (Python support is included in the monolithic USD lib).
    -- Python headers and the Python runtime lib are added manually below.
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
    "UsdTools.cpp","physicsSchemaTokens.cpp"
}

local python_module_cpp_files = {
    "module.cpp","moduleDeps.cpp","wrapUsdTools.cpp"
}

local python_module_files = {
    "__init__.py"
}

local resource_files = {
}

repo_build.prebuild_copy {
    { schema_source_dir.."/physicsSchemaTools/units.py", repo_build.target_dir().."/schema/lib/python/PhysicsSchemaTools" },
}

local python_options = { python_root = root.."/_build/target-deps/python" }

-- USD plugin C++ project
project("physicsSchemaTools")
    -- standard USD plugin settings
    usd_plugin.usd_plugin("physicsSchemaTools", options, public_headers, private_headers, cpp_files, resource_files, usd_libs)
    usd_plugin.use_standard_usd_options()
    do_usd_zcinline_fix()
    link_boost_for_windows_wdefault()
    -- Manually add Python support (headers + runtime lib) without the USD-specific python lib
    usd_base.use_python(python_options)
    -- Force-include nopy_compat.h: saturates the pxr.h include guard and un-defines
    -- PXR_PYTHON_SUPPORT_ENABLED so that no pxr_boost::python symbols appear in the DLL's
    -- import table.  This makes the DLL binary-compatible with both usd.py312 and usd.nopy.
    -- The _physicsSchemaTools Python extension is compiled separately and is unaffected.
    -- premake maps forceincludes to /FI on MSVC and -include on GCC/Clang.
    forceincludes { root.."/source/nopy_compat.h" }
    filter { "system:windows" }
        -- C4251: USD template members lack dll-interface — harmless, same class as in physxSchema.
        -- /external:W0 suppresses them for angle-bracket includes but not force-included contexts.
        disablewarnings { "4251" }
    filter {}
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
    -- Manually add Python support (headers + runtime lib) without the USD-specific python lib
    usd_base.use_python(python_options)
    -- For non-monolithic USD (namespaced USD 25.x), pxr_boost::python lives in usd_python.
    -- For monolithic USD (*usd_ms), those symbols are already inside the monolithic lib.
    link_usd_python_if_needed()
    filter { "system:linux", "configurations:debug" }
        libdirs { options.usd_root .. "/lib"}
        links { "tbb_debug" }
end
