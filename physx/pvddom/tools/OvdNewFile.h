// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once
#include <cstdio>
#include <cerrno>
#include <fcntl.h>
#ifdef _WIN32
#include <io.h>
#include <sys/stat.h>
#include <windows.h>
#include <string>
#else
#include <unistd.h>
#endif

#ifdef _WIN32
inline bool isAllowedNewOvdPath(const char* path)
{
    if (!path) return false;
    std::string normalized(path);
    for (char& character : normalized) if (character == '/') character = '\\';
    const auto driveLetter = [](char character) {
        return (character >= 'A' && character <= 'Z') || (character >= 'a' && character <= 'z');
    };
    size_t driveColon = std::string::npos;
    if (normalized.compare(0, 4, "\\\\?\\") == 0)
    {
        if (normalized.size() >= 7 && driveLetter(normalized[4]) && normalized[5] == ':' && normalized[6] == '\\')
            driveColon = 5;
        else if (normalized.size() < 8 ||
            (normalized[4] != 'U' && normalized[4] != 'u') ||
            (normalized[5] != 'N' && normalized[5] != 'n') ||
            (normalized[6] != 'C' && normalized[6] != 'c') || normalized[7] != '\\')
            return false;
    }
    else if (normalized.compare(0, 4, "\\\\.\\") == 0 ||
        normalized.compare(0, 4, "\\??\\") == 0 || normalized.compare(0, 5, "\\\\??\\") == 0)
        return false;
    else if (normalized.size() >= 2 && driveLetter(normalized[0]) && normalized[1] == ':')
        driveColon = 1;
    for (size_t index = 0; index < normalized.size(); ++index)
        if (normalized[index] == ':' && index != driveColon) return false;

    const size_t basenameStart = normalized.find_last_of('\\');
    const std::string basename = normalized.substr(basenameStart == std::string::npos ?
        (driveColon == 1 ? 2 : 0) : basenameStart + 1);
    const size_t dot = basename.find('.');
    const std::string device = basename.substr(0, dot);
    const auto upper = [](char character) { return character >= 'a' && character <= 'z' ? char(character - 'a' + 'A') : character; };
    std::string uppercase;
    for (char character : device) uppercase.push_back(upper(character));
    if (uppercase == "NUL" || uppercase == "CON" || uppercase == "PRN" || uppercase == "AUX" ||
        uppercase == "CONIN$" || uppercase == "CONOUT$") return false;
    if (uppercase.size() == 4 &&
        ((uppercase.compare(0, 3, "COM") == 0) || (uppercase.compare(0, 3, "LPT") == 0)) &&
        uppercase[3] >= '1' && uppercase[3] <= '9') return false;
    return true;
}
#endif

inline FILE* openNewOvdFile(const char* path)
{
#ifdef _WIN32
    if (!isAllowedNewOvdPath(path)) { errno = EINVAL; return nullptr; }
    errno = 0;

    const int fd = _open(path, _O_WRONLY | _O_CREAT | _O_EXCL | _O_BINARY, _S_IREAD | _S_IWRITE);
    if (fd < 0) return nullptr;
    const intptr_t handle = _get_osfhandle(fd);
    if (handle == -1 || GetFileType(reinterpret_cast<HANDLE>(handle)) != FILE_TYPE_DISK)
    {
        _close(fd);
        errno = EINVAL;
        return nullptr;
    }
    FILE* file = fd < 0 ? nullptr : _fdopen(fd, "wb");
    if (fd >= 0 && !file) _close(fd);
#else
    const int fd = open(path, O_WRONLY | O_CREAT | O_EXCL, 0600);
    FILE* file = fd < 0 ? nullptr : fdopen(fd, "wb");
    if (fd >= 0 && !file) close(fd);
#endif
    return file;
}
