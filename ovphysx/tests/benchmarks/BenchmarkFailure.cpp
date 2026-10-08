// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-BENCHMARK-001
 * @covers AC-4
 */

#include "BenchmarkFailure.h"

#include "framework/BmUtils.h"

#include <cstdarg>
#include <cstdio>
#include <string>
#include <vector>

namespace
{

struct FailureEntry
{
    std::string row;
    std::string detail;
};

// Function-local storage so there is no static init order dependency with the
// registration objects in the benchmark translation units.
std::vector<FailureEntry>& failures()
{
    static std::vector<FailureEntry> sFailures;
    return sFailures;
}

bool isHarnessPostfix(const std::string& suffix)
{
    if (suffix == "_GPU")
    {
        return true;
    }
    if (suffix.size() < 3 || suffix[0] != '_')
    {
        return false;
    }

    size_t pos = 1;
    if (suffix[pos] == '-')
    {
        ++pos;
    }
    const size_t digitStart = pos;
    while (pos < suffix.size() && suffix[pos] >= '0' && suffix[pos] <= '9')
    {
        ++pos;
    }
    if (pos == digitStart || pos >= suffix.size() || suffix[pos] != 'T')
    {
        return false;
    }
    ++pos;
    return pos == suffix.size() || suffix.compare(pos, std::string::npos, "_GPU") == 0;
}

} // namespace


void bmRecordFailure(const char* row, const char* format, ...)
{
    char buffer[1024];
    va_list args;
    va_start(args, format);
    std::vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    failures().push_back(FailureEntry{ row ? row : "<unknown>", buffer });

    // Still print immediately so the failure appears in pass output next to the
    // row it belongs to, not only in the trailing summary.
    printFormatted("FAILURE [%s] %s", row ? row : "<unknown>", buffer);
}


bool bmRowHasFailure(const char* registeredName)
{
    if (!registeredName)
    {
        return false;
    }
    const std::string name(registeredName);
    for (size_t i = 0; i < failures().size(); ++i)
    {
        const std::string& row = failures()[i].row;
        if (name == row)
        {
            return true;
        }
        if (name.size() > row.size() && name.compare(0, row.size(), row) == 0 && isHarnessPostfix(name.substr(row.size())))
        {
            return true;
        }
    }
    return false;
}


uint32_t bmFailureCount()
{
    return static_cast<uint32_t>(failures().size());
}


void bmPrintFailureSummary()
{
    if (failures().empty())
    {
        return;
    }
    printFormatted("");
    printFormatted("=== %u benchmark failure(s) ===", bmFailureCount());
    for (size_t i = 0; i < failures().size(); ++i)
    {
        printFormatted("  [%s] %s", failures()[i].row.c_str(), failures()[i].detail.c_str());
    }
    printFormatted("Operationally failed rows published no timing record.");
    printFormatted("A local non-regenerate baseline breach may already have printed its failed comparison.");
}
