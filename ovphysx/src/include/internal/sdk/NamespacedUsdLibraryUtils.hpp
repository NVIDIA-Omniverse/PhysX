// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <algorithm>
#include <cctype>
#include <fstream>
#include <string>
#include <vector>

#ifdef _WIN32
#    ifndef NOMINMAX
#        define NOMINMAX
#    endif
#    include <windows.h>
#    include <psapi.h>
#endif

namespace omni::sdk::internal
{

inline bool isNamespacedUsdLibraryFilename(std::string filename)
{
    std::transform(filename.begin(), filename.end(), filename.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });

#ifdef _WIN32
    return filename.rfind("ov_", 0) == 0 &&
           filename.find("usd_ms") != std::string::npos &&
           filename.size() >= 4 &&
           filename.substr(filename.size() - 4) == ".dll";
#else
    return filename.rfind("libov_", 0) == 0 &&
           filename.find("usd_ms") != std::string::npos &&
           filename.find(".so") != std::string::npos;
#endif
}

inline bool isNamespacedUsdLibraryPath(const std::string& path)
{
    const size_t lastSep = path.find_last_of("/\\");
    return isNamespacedUsdLibraryFilename(lastSep == std::string::npos ? path : path.substr(lastSep + 1));
}

#ifndef _WIN32
// Return the mapped file path from one /proc/self/maps line. The path field can
// contain spaces, so split only the fixed metadata fields before it.
inline std::string mapsLinePath(const std::string& line)
{
    size_t cursor = 0;
    for (int field = 0; field < 5; ++field)
    {
        cursor = line.find_first_not_of(" \t", cursor);
        if (cursor == std::string::npos)
        {
            return {};
        }
        cursor = line.find_first_of(" \t", cursor);
        if (cursor == std::string::npos)
        {
            return {};
        }
    }

    size_t pathStart = line.find_first_not_of(" \t", cursor);
    if (pathStart == std::string::npos || line[pathStart] != '/')
    {
        return {};
    }

    std::string fullPath = line.substr(pathStart);
    const std::string deletedSuffix = " (deleted)";
    if (fullPath.size() >= deletedSuffix.size() &&
        fullPath.compare(fullPath.size() - deletedSuffix.size(), deletedSuffix.size(), deletedSuffix) == 0)
    {
        fullPath.erase(fullPath.size() - deletedSuffix.size());
    }
    return fullPath;
}
#endif

// Find an OV namespaced USD monolith that is already loaded in this process.
// ovphysx uses this to share USD with peer OV libraries instead of mapping a
// second copy of the same namespaced runtime.
inline std::string findLoadedNamespacedUsdLibrary()
{
#ifdef _WIN32
    DWORD needed = 0;
    std::vector<HMODULE> modules(1024);
    if (!EnumProcessModules(GetCurrentProcess(), modules.data(), static_cast<DWORD>(modules.size() * sizeof(HMODULE)), &needed))
    {
        return {};
    }
    if (needed > modules.size() * sizeof(HMODULE))
    {
        modules.resize(needed / sizeof(HMODULE));
        if (!EnumProcessModules(GetCurrentProcess(), modules.data(),
                                static_cast<DWORD>(modules.size() * sizeof(HMODULE)), &needed))
        {
            return {};
        }
    }

    const DWORD count = needed / sizeof(HMODULE);
    for (DWORD i = 0; i < count; ++i)
    {
        char modulePath[MAX_PATH];
        if (GetModuleFileNameA(modules[i], modulePath, sizeof(modulePath)) &&
            isNamespacedUsdLibraryPath(modulePath))
        {
            return modulePath;
        }
    }
    return {};
#else
    std::ifstream maps("/proc/self/maps");
    if (!maps.is_open())
    {
        return {};
    }

    std::string line;
    while (std::getline(maps, line))
    {
        std::string fullPath = mapsLinePath(line);
        if (!fullPath.empty() && isNamespacedUsdLibraryPath(fullPath))
        {
            return fullPath;
        }
    }
    return {};
#endif
}

} // namespace omni::sdk::internal
