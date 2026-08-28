// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

// This file provides helper functions for resolving ovphysx runtime library and
// USD plugin paths, and for publishing the namespaced USD schema path to
// OV_PXR_PLUGINPATH_2511.

#include "UsdSchemaPaths/UsdSchemaPaths.h"

#include <cstdlib>
#include <filesystem>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

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

#ifdef _WIN32
static constexpr char kEnvPathSeparator = ';';
#else
static constexpr char kEnvPathSeparator = ':';
#endif

std::mutex g_registrationMutex;
std::mutex g_envMutex;
bool g_registrationDone = false;

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

std::string normalizePathForCompare(const std::string& path)
{
    return normalizeDirectoryPath(std::filesystem::path(path));
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

std::vector<std::string> splitEnvPaths(const std::string& value)
{
    std::vector<std::string> paths;
    size_t start = 0;
    while (start <= value.size())
    {
        size_t end = value.find(kEnvPathSeparator, start);
        if (end == std::string::npos)
        {
            end = value.size();
        }

        if (end > start)
        {
            paths.push_back(value.substr(start, end - start));
        }

        if (end == value.size())
        {
            break;
        }
        start = end + 1;
    }
    return paths;
}

bool pathAlreadyPresent(const std::vector<std::string>& paths, const std::string& candidate)
{
    const std::string normalizedCandidate = normalizePathForCompare(candidate);
    for (const std::string& path : paths)
    {
        if (normalizePathForCompare(path) == normalizedCandidate)
        {
            return true;
        }
    }
    return false;
}

bool setEnvironmentVariable(const char* name, const std::string& value)
{
#ifdef _WIN32
    return _putenv_s(name, value.c_str()) == 0;
#else
    return setenv(name, value.c_str(), 1) == 0;
#endif
}

std::string makeMissingRootError(const std::string& candidate)
{
    std::ostringstream ss;
    ss << "Failed to register ovphysx USD schema/plugin paths: expected an existing directory at ";
    if (candidate.empty())
    {
        ss << "<empty path>";
    }
    else
    {
        ss << candidate;
    }
    ss << ". Set OVPHYSX_LIB to the ovphysx shared library path or install the ovphysx SDK/wheel layout with plugins/usd present.";
    return ss.str();
}

} // namespace

std::string getLibraryDirectory()
{
    if (std::string envOverrideDir = getLibraryDirectoryFromEnvOverride(); !envOverrideDir.empty())
    {
        return envOverrideDir;
    }
    return getLoadedOvphysxDirectory();
}

std::string getPluginsDirectory()
{
    std::string libDir = getLibraryDirectory();
    if (libDir.empty())
    {
        return "";
    }

    const std::filesystem::path libraryDirPath(libDir);

    const std::filesystem::path sdkLayoutPluginsPath = libraryDirPath.parent_path() / "plugins";
    std::error_code ec;
    auto hasUsdRegistry = [](const std::filesystem::path& pluginsPath) {
        std::error_code usdEc;
        return std::filesystem::is_directory(pluginsPath / "usd", usdEc) && !usdEc;
    };

    const std::filesystem::path copiedRuntimePluginsPath = libraryDirPath / "plugins";
    if (hasUsdRegistry(sdkLayoutPluginsPath))
    {
        return normalizeDirectoryPath(sdkLayoutPluginsPath);
    }
    if (hasUsdRegistry(copiedRuntimePluginsPath))
    {
        return normalizeDirectoryPath(copiedRuntimePluginsPath);
    }
    if (std::filesystem::is_directory(sdkLayoutPluginsPath, ec) && !ec)
    {
        return normalizeDirectoryPath(sdkLayoutPluginsPath);
    }
    ec.clear();
    if (std::filesystem::is_directory(copiedRuntimePluginsPath, ec) && !ec)
    {
        return normalizeDirectoryPath(copiedRuntimePluginsPath);
    }

    return normalizeDirectoryPath(sdkLayoutPluginsPath);
}

std::string getUsdPluginsDirectory()
{
    std::string pluginsDir = getPluginsDirectory();
    if (pluginsDir.empty())
    {
        return "";
    }

    return normalizeDirectoryPath(std::filesystem::path(pluginsDir) / "usd");
}

bool registerSchemaPaths(std::string* out_error)
{
    std::lock_guard<std::mutex> lock(g_envMutex);

    const std::string usdPluginPath = getUsdPluginsDirectory();
    std::error_code ec;
    if (usdPluginPath.empty() || !std::filesystem::is_directory(usdPluginPath, ec) || ec)
    {
        if (out_error)
        {
            *out_error = makeMissingRootError(usdPluginPath);
        }
        return false;
    }

    const char* existing = std::getenv(kNamespacedUsdPluginPathEnvVar);
    std::string envValue = existing ? existing : "";
    std::vector<std::string> paths = splitEnvPaths(envValue);
    const std::string normalizedUsdPluginPath = normalizeDirectoryPath(usdPluginPath);

    if (!pathAlreadyPresent(paths, normalizedUsdPluginPath))
    {
        if (!envValue.empty() && envValue.back() != kEnvPathSeparator)
        {
            envValue.push_back(kEnvPathSeparator);
        }
        envValue += normalizedUsdPluginPath;
        if (!setEnvironmentVariable(kNamespacedUsdPluginPathEnvVar, envValue))
        {
            if (out_error)
            {
                *out_error = "Failed to update OV_PXR_PLUGINPATH_2511 for ovphysx USD schema/plugin paths";
            }
            return false;
        }
    }

    if (out_error)
    {
        out_error->clear();
    }
    return true;
}

bool registerSchemaPathsOnce(std::string* out_error, bool* out_registered)
{
    std::lock_guard<std::mutex> lock(g_registrationMutex);
    if (g_registrationDone)
    {
        if (out_error)
        {
            out_error->clear();
        }
        if (out_registered)
        {
            *out_registered = false;
        }
        return true;
    }

    if (!registerSchemaPaths(out_error))
    {
        if (out_registered)
        {
            *out_registered = false;
        }
        return false;
    }

    g_registrationDone = true;
    if (out_registered)
    {
        *out_registered = true;
    }
    return true;
}

void resetSchemaPathRegistrationForTests()
{
    std::lock_guard<std::mutex> lock(g_registrationMutex);
    g_registrationDone = false;
}

} // namespace usd_schema_paths
} // namespace sdk
} // namespace omni
