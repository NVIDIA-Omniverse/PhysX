// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "../tools/OvdNewFile.h"
#include <windows.h>
#include <cstdio>
#include <cstring>
#include <string>

int main()
{
    SetErrorMode(SEM_FAILCRITICALERRORS | SEM_NOOPENFILEERRORBOX);
    char temporary[MAX_PATH];
    char directory[MAX_PATH];
    const DWORD pathLength = GetTempPathA(MAX_PATH, temporary);
    if (!pathLength || pathLength >= MAX_PATH || !GetTempFileNameA(temporary, "ovd", 0, directory)) return 1;
    if (!DeleteFileA(directory) || !CreateDirectoryA(directory, nullptr)) return 1;
    const std::string source = std::string(directory) + "\\source.ovd";
    const std::string evidence = std::string(directory) + "\\evidence.json";
    const std::string extended = std::string("\\\\?\\") + directory + "\\extended.json";
    int result = 0;
    const auto fail = [&](const char* reason) { std::fprintf(stderr, "%s\n", reason); result = 1; };
    struct PathCase { const char* path; bool allowed; };
    const PathCase cases[] = {
        {"capture.ovd", true}, {"C:\\captures\\capture.ovd", true},
        {"\\\\server\\share\\capture.ovd", true},
        {"\\\\?\\C:\\captures\\capture.ovd", true},
        {"\\\\?\\UNC\\server\\share\\capture.ovd", true},
        {"capture.ovd:evidence", false}, {"C:\\captures\\capture.ovd:evidence", false},
        {"NUL", false}, {"CON", false}, {"COM9", false}, {"C:NUL", false},
        {"C:\\captures\\LPT1.txt", false}, {"\\\\?\\Volume{1234}\\capture.ovd", false},
        {"\\\\.\\PhysicalDrive0", false}, {"\\??\\C:\\capture.ovd", false},
        {"\\\\??\\C:\\capture.ovd", false},
        {"\\??\\UNC\\server\\share\\capture.ovd", false},
        {"\\\\??\\UNC\\server\\share\\capture.ovd", false},
        {"PRN", false}, {"AUX", false}, {"CONIN$", false},
        {"conout$.json", false}, {"nul", false}, {"NUL.tar.gz", false}
    };
    for (const auto& pathCase : cases)
        if (isAllowedNewOvdPath(pathCase.path) != pathCase.allowed) fail(pathCase.path);
    if (isAllowedNewOvdPath(nullptr)) fail("null path was allowed");
    FILE* file = openNewOvdFile(source.c_str());
    if (!file || std::fwrite("source", 1, 6, file) != 6) fail("ordinary create failed");
    if (file) std::fclose(file);

    WIN32_FILE_ATTRIBUTE_DATA before = {};
    WIN32_FILE_ATTRIBUTE_DATA after = {};
    if (!GetFileAttributesExA(source.c_str(), GetFileExInfoStandard, &before)) fail("source metadata unavailable");
    file = openNewOvdFile(source.c_str());
    if (file) { std::fclose(file); fail("existing file opened"); }
    file = openNewOvdFile((source + ":evidence").c_str());
    if (file) { std::fclose(file); fail("alternate stream opened"); }
    if (errno != EINVAL) fail("alternate stream did not set EINVAL");
    HANDLE stream = CreateFileA((source + ":evidence").c_str(), GENERIC_READ,
        FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (stream != INVALID_HANDLE_VALUE) { CloseHandle(stream); fail("alternate stream exists"); }
    if (!GetFileAttributesExA(source.c_str(), GetFileExInfoStandard, &after) ||
        std::memcmp(&before.ftLastWriteTime, &after.ftLastWriteTime, sizeof(FILETIME)) != 0) fail("source timestamp changed");
    char contents[7] = {};
    file = std::fopen(source.c_str(), "rb");
    if (!file || std::fread(contents, 1, 6, file) != 6 || std::strcmp(contents, "source") != 0) fail("source content changed");
    if (file) std::fclose(file);

    file = openNewOvdFile("NUL");
    if (file) { std::fclose(file); fail("NUL opened"); }
    errno = 0;
    file = openNewOvdFile("NUL ");
    if (file) { std::fclose(file); fail("trailing-space device opened"); }
    if (errno != EINVAL) fail("post-open device refusal did not set EINVAL");
    file = openNewOvdFile("\\\\.\\NUL");
    if (file) { std::fclose(file); fail("device namespace opened"); }
    if (errno != EINVAL) fail("device namespace did not set EINVAL");
    file = openNewOvdFile(evidence.c_str());
    if (!file) fail("ordinary evidence create failed");
    if (file) std::fclose(file);
    file = openNewOvdFile(extended.c_str());
    if (!file) fail("extended drive create failed");
    if (file) std::fclose(file);

    DeleteFileA((std::string(directory) + "\\extended.json").c_str());
    DeleteFileA(evidence.c_str());
    DeleteFileA(source.c_str());
    RemoveDirectoryA(directory);
    return result;
}
