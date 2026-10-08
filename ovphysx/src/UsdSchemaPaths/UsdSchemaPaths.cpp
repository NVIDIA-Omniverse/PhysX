// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-OVSTAGE-SCHEMA-001
 * @covers AC-1 AC-2
 */

// Filesystem discovery for the ovphysx runtime layout: the library directory,
// the Carbonite plugin directory, and the codeless PhysX USD schema root. No
// OpenUSD code is involved and nothing here modifies the process environment.

#include "UsdSchemaPaths/UsdSchemaPaths.h"

#include <cstdlib>
#include <filesystem>
#include <string>

#ifdef _WIN32
    #ifndef NOMINMAX
        #define NOMINMAX
    #endif
    #include <windows.h>
#else
    #include <dlfcn.h>
#endif

namespace omni {
namespace sdk {
namespace usd_schema_paths {

namespace {

std::string normalizeDirectoryPath(std::filesystem::path path)
{
    if (path.empty())
    {
        return "";
    }

    path.make_preferred();
    std::error_code ec;
    std::filesystem::path normalized = std::filesystem::weakly_canonical(path, ec);
    if (!ec)
    {
        normalized.make_preferred();
        return normalized.string();
    }

    path = path.lexically_normal();
    path.make_preferred();
    return path.string();
}

std::string getLibraryDirectoryFromEnvOverride()
{
    const char* libOverride = std::getenv("OVPHYSX_LIB");
    if (!libOverride || libOverride[0] == '\0')
    {
        return "";
    }

    std::filesystem::path overridePath(libOverride);
    std::error_code ec;
    if (!overridePath.is_absolute())
    {
        overridePath = std::filesystem::absolute(overridePath, ec);
        if (ec)
        {
            return "";
        }
    }

    if (std::filesystem::is_directory(overridePath, ec) && !ec)
    {
        return normalizeDirectoryPath(overridePath);
    }

    std::filesystem::path dir = overridePath.parent_path();
    if (dir.empty())
    {
        return "";
    }
    return normalizeDirectoryPath(dir);
}

std::string getLoadedOvphysxDirectory()
{
#ifdef _WIN32
    HMODULE hModule = NULL;
    if (GetModuleHandleExA(
            GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
            reinterpret_cast<LPCSTR>(&getLibraryDirectory),
            &hModule))
    {
        char path[MAX_PATH];
        if (GetModuleFileNameA(hModule, path, MAX_PATH))
        {
            std::filesystem::path fullPath(path);
            return normalizeDirectoryPath(fullPath.parent_path());
        }
    }
    return "";
#else
    Dl_info dlInfo;
    if (dladdr(reinterpret_cast<void*>(&getLibraryDirectory), &dlInfo))
    {
        std::filesystem::path fullPath(dlInfo.dli_fname);
        return normalizeDirectoryPath(fullPath.parent_path());
    }
    return "";
#endif
}

bool isDirectory(const std::filesystem::path& path)
{
    std::error_code ec;
    return std::filesystem::is_directory(path, ec) && !ec;
}

bool isRegularFile(const std::filesystem::path& path)
{
    std::error_code ec;
    return std::filesystem::is_regular_file(path, ec) && !ec;
}

// A plugins/ directory is recognized by a Carbonite plugin that every ovphysx
// layout ships, so an unrelated plugins/ directory one level above a copied
// runtime cannot shadow the real one.
bool looksLikeOvphysxPluginsDirectory(const std::filesystem::path& plugins)
{
#ifdef _WIN32
    return isRegularFile(plugins / "carb.datastore.plugin.dll");
#else
    return isRegularFile(plugins / "libcarb.datastore.plugin.so");
#endif
}

} // namespace

std::string getLibraryDirectory()
{
    std::string envOverrideDir = getLibraryDirectoryFromEnvOverride();
    if (!envOverrideDir.empty())
    {
        return envOverrideDir;
    }
    return getLoadedOvphysxDirectory();
}

std::string getPluginsDirectory()
{
    const std::string libDir = getLibraryDirectory();
    if (libDir.empty())
    {
        return "";
    }

    const std::filesystem::path libraryDirPath(libDir);
    const std::filesystem::path sdkLayoutPluginsPath = libraryDirPath.parent_path() / "plugins";
    const std::filesystem::path copiedRuntimePluginsPath = libraryDirPath / "plugins";

    // Next to the library first (copied runtime), then one level up (SDK
    // layout, whose lib/ holds no plugins/). A recognized ovphysx plugin
    // directory wins over a bare directory of the same name.
    if (looksLikeOvphysxPluginsDirectory(copiedRuntimePluginsPath))
    {
        return normalizeDirectoryPath(copiedRuntimePluginsPath);
    }
    if (looksLikeOvphysxPluginsDirectory(sdkLayoutPluginsPath))
    {
        return normalizeDirectoryPath(sdkLayoutPluginsPath);
    }
    if (isDirectory(copiedRuntimePluginsPath))
    {
        return normalizeDirectoryPath(copiedRuntimePluginsPath);
    }
    if (isDirectory(sdkLayoutPluginsPath))
    {
        return normalizeDirectoryPath(sdkLayoutPluginsPath);
    }

    return normalizeDirectoryPath(sdkLayoutPluginsPath);
}

std::string getCodelessSchemaRoot(std::string* out_error)
{
    const std::string libDir = getLibraryDirectory();
    if (libDir.empty())
    {
        if (out_error)
        {
            *out_error = "Failed to locate the ovphysx codeless schemas: the ovphysx library directory "
                         "could not be determined. Set OVPHYSX_LIB to the ovphysx shared library path.";
        }
        return "";
    }

    const std::filesystem::path libraryDirPath(libDir);
    // Next to the library first (a runtime copied beside an application), then
    // one level up (the SDK and wheel layouts, whose lib/ holds no schemas/).
    // An unrelated schemas/ tree above a copied runtime therefore never wins.
    const std::filesystem::path candidates[] = {
        libraryDirPath / "schemas" / "physx",
        libraryDirPath.parent_path() / "schemas" / "physx",
    };
    for (const std::filesystem::path& candidate : candidates)
    {
        if (isRegularFile(candidate / "plugInfo.json"))
        {
            if (out_error)
            {
                out_error->clear();
            }
            return normalizeDirectoryPath(candidate);
        }
    }

    if (out_error)
    {
        *out_error = "Failed to locate the ovphysx codeless schemas: no schemas/physx/plugInfo.json found "
                     "next to the ovphysx library (checked " + candidates[0].string() + " and " +
                     candidates[1].string() + "). Set OVPHYSX_LIB to the ovphysx shared library path or "
                     "reinstall the ovphysx SDK/wheel.";
    }
    return "";
}

#ifdef _WIN32
std::string getLoadedLibraryDirectory(const std::string& libraryName)
{
    HMODULE hModule = GetModuleHandleA(libraryName.c_str());
    if (hModule == NULL)
    {
        return "";
    }
    char path[MAX_PATH];
    if (GetModuleFileNameA(hModule, path, MAX_PATH) == 0)
    {
        return "";
    }
    return normalizeDirectoryPath(std::filesystem::path(path).parent_path());
}
#endif

} // namespace usd_schema_paths
} // namespace sdk
} // namespace omni
