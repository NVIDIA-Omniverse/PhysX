// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <filesystem>
#include <functional>
#include <string>

namespace omni::sdk::internal
{
    using FilesystemErrorHandler = std::function<void(
        const std::filesystem::path&,
        const char*,
        const std::filesystem::filesystem_error&)>;

/**
 * Linux/Unix-only helper for locating .so libraries.
 *
 * This function searches for a library by its exact name or by a versioned
     * pattern. It first attempts a direct path join. If not found, it iterates
     * through the directory to find a file starting with the library's stem
     * name followed by a hyphen (e.g., "libusd-") and ending in ".so".
     *
     * @param baseDir The directory path to search within.
     * @param libName The name of the library to find (e.g., "libusd.so").
     * @return std::string The absolute or relative path to the library if found;
     * otherwise, an empty string.
     * @note This implementation is specific to Linux/Unix systems (.so extension).
     */
    inline std::string findLibPath(
        const std::string& baseDir,
        const char* libName,
        const FilesystemErrorHandler& onError = {})
    {
        if (baseDir.empty())
            return {};
        std::filesystem::path dir(baseDir);
        if (!std::filesystem::exists(dir))
            return {};
        std::filesystem::path direct = dir / libName;
        if (std::filesystem::exists(direct))
            return direct.string();
        std::string stem = std::filesystem::path(libName).stem().string();
        try
        {
            for (const auto& entry : std::filesystem::directory_iterator(dir))
            {
                if (!entry.is_regular_file())
                    continue;
                const std::string filename = entry.path().filename().string();
                if (filename.rfind(stem + "-", 0) == 0 && entry.path().extension() == ".so")
                    return entry.path().string();
            }
        }
        catch (const std::filesystem::filesystem_error& e)
        {
            if (onError)
                onError(dir, libName, e);
        }
        return {};
    }
} // namespace omni::sdk::internal
