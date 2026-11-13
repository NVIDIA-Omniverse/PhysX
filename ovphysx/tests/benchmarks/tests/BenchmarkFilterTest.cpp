// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "../framework/BmFilter.h"

#include <cstdio>
#include <string>

namespace
{

struct FilterCase
{
    const char* filter;
    const char* name;
    bool expected;
};

bool expect(bool condition, const char* message)
{
    if (!condition)
        std::fprintf(stderr, "%s\n", message);
    return condition;
}

} // namespace

int main()
{
    const FilterCase cases[] = {
        { nullptr, "Step.cpu", true },
        { nullptr, "", true },
        { "", "", true },
        { "", "Step.cpu", false },
        { "Step.cpu", "Step.cpu", true },
        { "Step.cpu", "Step.cpu.extra", false },
        { "Step.cpu", "prefix.Step.cpu", false },
        { "Step.cpu", "step.cpu", false },
        { "*", "", true },
        { "*", "Step.cpu", true },
        { "?", "", false },
        { "?", "a", true },
        { "?", "ab", false },
        { "Step.?pu", "Step.cpu", true },
        { "Step.?pu", "Step.gpu", true },
        { "Step.?pu", "Step.pu", false },
        { "Step.*", "Step.", true },
        { "Step.*", "Step.cpu", true },
        { "Step.*", "Other.cpu", false },
        { "*a*b", "aaab", true },
        { "*a*b", "aaac", false },
        { "a**?*b", "acb", true },
        { "a**?*b", "ab", false },
        { "***", "abc", true },
        { "*a", "*ba", true },
        { "a?b", "a*b", true },
        { "a?b", "a:b", true },
        { "[a]", "a", false },
        { "[a]", "[a]", true },
        { "a\\b", "a\\b", true },
        { "foo:bar", "foo", true },
        { "foo:bar", "bar", true },
        { "foo:bar", "foo:bar", false },
        { "foo:bar", "other", false },
        { "*:-*.gpu", "Step.cpu", true },
        { "*:-*.gpu", "Step.gpu", false },
        { "-*.gpu:*", "Step.gpu", false },
        { "-*.gpu", "Step.cpu", true },
        { "-*.gpu", "Step.gpu", false },
        { "-foo:-bar", "other", true },
        { "-foo:bar", "other", false },
        { "-foo:bar", "bar", true },
        { "foo:-foo", "foo", false },
        { "-foo:foo", "foo", false },
        { "foo:", "", true },
        { "foo:", "foo", true },
        { ":foo", "", true },
        { "::", "x", false },
        { "::", "", true },
        { "-foo:", "other", false },
        { "-foo:", "", true },
        { "-", "", false },
        { "-", "x", true },
        { "--foo", "-foo", false },
        { "--foo", "foo", true },
        { "a\0:other", "a\0hidden", true },
        { "?", "\0hidden", false },
    };
    bool ok = true;
    for (const FilterCase& test : cases)
    {
        const bool actual = BenchmarkHarness::matchFilter(test.filter, test.name);
        if (actual != test.expected)
        {
            std::fprintf(stderr, "filter=%s name=%s expected=%d actual=%d\n",
                         test.filter ? test.filter : "<null>", test.name, test.expected, actual);
            ok = false;
        }
    }

    // A recursive matcher consumes stack proportional to this name's length.
    const std::string longName(100000, 'a');
    ok = expect(BenchmarkHarness::matchFilter("*a", longName.c_str()), "long name did not match") && ok;
    ok = expect(!BenchmarkHarness::matchFilter("*b", longName.c_str()), "long name matched wrong suffix") && ok;

    // A failing suffix after many stars must not explore every partition.
    const std::string manyStars = std::string(200, '*') + 'b';
    const std::string shortName(200, 'a');
    ok = expect(!BenchmarkHarness::matchFilter(manyStars.c_str(), shortName.c_str()),
                "star-heavy failing pattern matched") && ok;
    const std::string suffixPattern = '*' + std::string(512, 'a') + 'b';
    const std::string repeatedName(4096, 'a');
    ok = expect(!BenchmarkHarness::matchFilter(suffixPattern.c_str(), repeatedName.c_str()),
                "repeated failing suffix matched") && ok;

    if (ok)
        std::printf("Passed %zu filter cases and 4 long/backtracking checks.\n", sizeof(cases) / sizeof(cases[0]));
    return ok ? 0 : 1;
}
