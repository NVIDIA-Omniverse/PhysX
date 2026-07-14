-- Minimal premake5-public.lua retained for the schema build (omni/schema/).
-- The ovexts build itself has moved to CMake; this file only provides the
-- constants and helpers that omni/schema/premake5.lua still includes.

MSVC_VERSION = "14.29.30133"
WINSDK_VERSION = "10.0.19041.0"

function link_boost_for_windows_wdefault(libs)
    --- just for macros now
    defines { "BOOST_ALL_NO_LIB", "BOOST_ALL_DYN_LINK" }
end

-- Link usd_python for non-monolithic USD builds.
-- Monolithic builds (single *usd_ms lib) bundle all pxr_boost::python symbols.
-- Non-monolithic builds (namespaced USD 25.x) ship them in a separate usd_python lib.
-- Mirrors the logic in omni/ovruntime/cmake/UsdLinkDependencies.cmake.
function link_usd_python_if_needed()
    local ms_libs = {}
    for _, cfg in ipairs({"release", "debug"}) do
        local usd_lib_dir = root.."/_build/target-deps/usd/"..cfg.."/lib"
        ms_libs = os.matchfiles(usd_lib_dir.."/*usd_ms.*")
        if #ms_libs > 0 then
            break
        end
    end
    if #ms_libs == 0 then
        links { "usd_python" }
    end
end

function do_usd_zcinline_fix()
    filter { "system:windows" }
    filter { "toolset:msc-v142" }
    buildoptions { removeunreferencedcodedata ("off") }
    filter {}
end
