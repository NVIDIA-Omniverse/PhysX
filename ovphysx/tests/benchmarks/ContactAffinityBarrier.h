// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-BENCHMARK-004
 * @covers AC-6
 */

#ifndef CONTACT_AFFINITY_BARRIER_H
#define CONTACT_AFFINITY_BARRIER_H

#include <charconv>
#include <chrono>
#include <cstring>
#include <stdexcept>

#if defined(__linux__)
#include <cerrno>
#include <poll.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

// Test-runner synchronization only. Affinity policy belongs to the external
// runner; no runtime API or scene coordination behavior changes here.
class ContactAffinityBarrier
{
public:
    explicit ContactAffinityBarrier(const char* descriptors)
    {
        if (!descriptors)
            return;
#if defined(__linux__)
        const char* separator = std::strchr(descriptors, ':');
        const char* end = descriptors + std::strlen(descriptors);
        if (!separator)
            throw std::invalid_argument("contact affinity requires notify:release pipe descriptors");
        const std::from_chars_result first = std::from_chars(descriptors, separator, mNotify);
        const std::from_chars_result second = std::from_chars(separator + 1, end, mRelease);
        if (first.ec != std::errc() || first.ptr != separator || second.ec != std::errc() ||
            second.ptr != end || mNotify < 3 || mRelease < 3 || mNotify == mRelease)
            throw std::invalid_argument("invalid contact affinity pipe descriptors");
        struct stat notifyStat{}, releaseStat{};
        if (fstat(mNotify, &notifyStat) != 0 || fstat(mRelease, &releaseStat) != 0 ||
            !S_ISFIFO(notifyStat.st_mode) || !S_ISFIFO(releaseStat.st_mode))
            throw std::invalid_argument("contact affinity descriptors must be open pipes");
#else
        throw std::invalid_argument("contact affinity synchronization requires Linux");
#endif
    }

    void exchange(char phase) const
    {
        if (mNotify < 0)
            return;
#if defined(__linux__)
        ssize_t written;
        do
        {
            written = write(mNotify, &phase, 1);
        } while (written < 0 && errno == EINTR);
        if (written != 1)
            throw std::runtime_error("contact affinity runner notification failed");
        const std::chrono::steady_clock::time_point deadline =
            std::chrono::steady_clock::now() + std::chrono::seconds(30);
        for (;;)
        {
            const std::chrono::milliseconds::rep remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now()).count();
            if (remaining <= 0)
                throw std::runtime_error("contact affinity runner timed out");
            pollfd wait{ mRelease, POLLIN, 0 };
            const int result = poll(&wait, 1, static_cast<int>(remaining));
            if (result < 0 && errno == EINTR)
                continue;
            if (result <= 0)
                throw std::runtime_error("contact affinity runner wait failed");
            char acknowledgement = 0;
            ssize_t received;
            do
            {
                received = read(mRelease, &acknowledgement, 1);
            } while (received < 0 && errno == EINTR);
            if (received != 1 || acknowledgement != 'A')
                throw std::runtime_error("contact affinity runner disconnected or rejected placement");
            return;
        }
#else
        (void)phase;
#endif
    }

private:
    int mNotify = -1;
    int mRelease = -1;
};

#endif
