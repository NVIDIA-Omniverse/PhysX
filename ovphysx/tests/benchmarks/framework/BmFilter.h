// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef BENCHMARK_FILTER_H
#define BENCHMARK_FILTER_H

#include <cstddef>
#include <string_view>

namespace BenchmarkHarness
{

// Same iterative algorithm as the runtime's PathPatternMatcher.cpp. Retry the
// most recent star without recursion or allocations: O(pattern * name) worst
// case time and constant auxiliary space.
inline bool matchSimpleGlob(std::string_view pattern, std::string_view name)
{
    if (pattern.find_first_of("*?") == std::string_view::npos)
        return pattern == name;
    if (pattern == "*")
        return name.find('\0') == std::string_view::npos;

    std::size_t p = 0;
    std::size_t n = 0;
    std::size_t star = std::string_view::npos;
    std::size_t retry = 0;
    while (n < name.size())
    {
        if (p < pattern.size() && pattern[p] == '*')
        {
            star = p++;
            retry = n;
        }
        else if (p < pattern.size() &&
                 ((pattern[p] == '?' && name[n] != '\0') || pattern[p] == name[n]))
        {
            ++p;
            ++n;
        }
        else if (star != std::string_view::npos && retry < name.size() && name[retry] != '\0')
        {
            p = star + 1;
            n = ++retry;
        }
        else
            return false;
    }
    while (p < pattern.size() && pattern[p] == '*')
        ++p;
    return p == pattern.size();
}

// A null filter selects every name. Otherwise, ':' separates patterns and a
// leading '-' excludes a pattern. Any exclusion wins; an exclusion-only list
// includes every name that it does not exclude. Empty tokens remain patterns.
inline bool matchFilter(const char* filter, const char* name)
{
    if (!filter)
        return true;

    // Capture the C-string bounds once; token parsing stays within these views.
    std::string_view remaining(filter);
    const std::string_view nameView(name);
    bool match = false;
    bool noIncludeFilter = true;
    for (;;)
    {
        const bool exclude = !remaining.empty() && remaining.front() == '-';
        if (exclude)
            remaining.remove_prefix(1);
        const std::size_t next = remaining.find(':');

        if (matchSimpleGlob(remaining.substr(0, next), nameView))
        {
            if (exclude)
                return false;
            match = true;
        }

        noIncludeFilter &= exclude;
        if (next == std::string_view::npos)
            break;
        remaining.remove_prefix(next + 1);
    }
    return match || noIncludeFilter;
}

} // namespace BenchmarkHarness

#endif
