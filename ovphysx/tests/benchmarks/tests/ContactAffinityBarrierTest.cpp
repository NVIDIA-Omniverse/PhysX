// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "../ContactAffinityBarrier.h"

#include <cstdio>
#include <string>
#include <sys/wait.h>

static void require(bool condition)
{
    if (!condition)
        throw std::runtime_error("contact affinity barrier test failed");
}

static void exercise(bool disconnect)
{
    int notification[2], release[2];
    require(pipe(notification) == 0 && pipe(release) == 0);
    const pid_t child = fork();
    require(child >= 0);
    if (child == 0)
    {
        close(notification[0]);
        close(release[1]);
        const std::string descriptors = std::to_string(notification[1]) + ":" + std::to_string(release[0]);
        try
        {
            ContactAffinityBarrier barrier(descriptors.c_str());
            barrier.exchange('R');
            barrier.exchange('E');
            _exit(0);
        }
        catch (const std::exception&)
        {
            _exit(1);
        }
    }
    close(notification[1]);
    close(release[0]);
    char event = 0;
    require(read(notification[0], &event, 1) == 1 && event == 'R');
    int status = 0;
    require(waitpid(child, &status, WNOHANG) == 0);
    if (!disconnect)
    {
        require(write(release[1], "A", 1) == 1);
        require(read(notification[0], &event, 1) == 1 && event == 'E');
        require(write(release[1], "A", 1) == 1);
    }
    close(release[1]);
    close(notification[0]);
    require(waitpid(child, &status, 0) == child && WIFEXITED(status));
    require(WEXITSTATUS(status) == (disconnect ? 1 : 0));
}

int main()
{
    ContactAffinityBarrier disabled(nullptr);
    disabled.exchange('R');
    for (const char* invalid : { "", "1:2", "3", "3:3", "-1:4", "3:4junk" })
    {
        bool rejected = false;
        try
        {
            ContactAffinityBarrier barrier(invalid);
        }
        catch (const std::invalid_argument&)
        {
            rejected = true;
        }
        require(rejected);
    }
    exercise(false);
    exercise(true);
    std::puts("Contact affinity barrier tests passed");
    return 0;
}
