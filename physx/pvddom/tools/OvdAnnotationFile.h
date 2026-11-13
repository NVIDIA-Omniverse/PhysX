// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "OvdNewFile.h"
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>

template<typename Write, typename Close, typename Remove>
void saveNewOvdAnnotations(const std::string& path, const std::string& text, Write write, Close close, Remove remove)
{
    FILE* file = openNewOvdFile(path.c_str());
    if (!file)
    {
        const int error = errno ? errno : EIO;
        throw std::runtime_error("Cannot exclusively create annotations: " + path + ": " + std::strerror(error));
    }
    const bool written = write(file, text);
    const int writeError = written ? 0 : errno;
    const bool closed = close(file);
    const int closeError = closed ? 0 : errno;
    if (!written || !closed)
    {
        const int originalError = writeError ? writeError : closeError ? closeError : EIO;
        const bool removed = remove(path.c_str()) == 0;
        const int removeError = removed ? 0 : errno;
        const std::string diagnostic = "Could not finalize annotations: " + path + ": " +
            std::strerror(originalError) + (removed ? "" :
                "; could not remove partial annotations: " + std::string(std::strerror(removeError ? removeError : EIO)));
        errno = originalError;
        throw std::runtime_error(diagnostic);
    }
}

template<typename Write, typename Close>
void saveNewOvdAnnotations(const std::string& path, const std::string& text, Write write, Close close)
{
    saveNewOvdAnnotations(path, text, write, close,
        [](const char* filename) { return std::remove(filename); });
}
