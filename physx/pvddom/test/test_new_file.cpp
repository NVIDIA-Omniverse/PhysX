// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "../tools/OvdNewFile.h"
#include <cstdio>
#include <cstdlib>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

int main(int argc, char** argv)
{
    if (argc != 2) return 2;
    std::string directoryTemplate = std::string(argv[1]) + "/ovd-new-file-XXXXXX";
    const char* directory = mkdtemp(&directoryTemplate[0]);
    if (!directory)
    {
        std::perror("mkdtemp");
        return 1;
    }

    const std::string path = std::string(directory) + "/capture.ovd";
    const mode_t previousUmask = umask(0);
    FILE* file = openNewOvdFile(path.c_str());
    umask(previousUmask);

    int result = 0;
    if (!file)
    {
        std::perror("openNewOvdFile");
        result = 1;
    }
    else
    {
        if (std::fclose(file) != 0)
        {
            std::perror("fclose");
            result = 1;
        }
        struct stat details;
        if (stat(path.c_str(), &details) != 0)
        {
            std::perror("stat");
            result = 1;
        }
        else if ((details.st_mode & 0777) != 0600)
        {
            std::fprintf(stderr, "New capture mode was %03o, expected 600\n", details.st_mode & 0777);
            result = 1;
        }
        FILE* existing = openNewOvdFile(path.c_str());
        if (existing)
        {
            std::fprintf(stderr, "Existing capture was opened for writing\n");
            std::fclose(existing);
            result = 1;
        }
    }

    if (unlink(path.c_str()) != 0 && file)
    {
        std::perror("unlink");
        result = 1;
    }
    if (rmdir(directory) != 0)
    {
        std::perror("rmdir");
        result = 1;
    }
    return result;
}
