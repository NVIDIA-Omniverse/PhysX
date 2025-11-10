// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <common/utilities/PhysXErrorCallback.h>
#include <iostream>
#include <regex>

bool CarbPhysXErrorCallback::getCustomErrMsg(const std::string& msg, std::string& customErrMsg)
{
    constexpr size_t fmtBufferSize = 1024;
    std::smatch matches;

    auto regex = std::regex("^(The application needs to increase )(\\S+)(.*) to (\\d+)(.*)$", std::regex::icase);

    if (std::regex_search(msg, matches, regex) && matches.size() == 6)
    {
        const std::unordered_map<std::string, std::string> tranDict = {
            { "PxgDynamicsMemoryConfig::foundLostAggregatePairsCapacity", "Gpu Found Lost Aggregate Pairs Capacity" },
            { "PxgDynamicsMemoryConfig::totalAggregatePairsCapacity", "Gpu Total Aggregate Pairs Capacity" },
        };

        auto item = tranDict.find(matches[2].str());
        if (item != tranDict.end())
        {
            char fmtBuffer[fmtBufferSize];

            snprintf(fmtBuffer, sizeof(fmtBuffer),
                     "Please select Physics Scene (create one if there is none), navigate to GPU section and increase \"%s\" value to at least %s.\n\nMore details:\n%s",
                     item->second.c_str(), matches[4].str().c_str(), msg.c_str());

            customErrMsg = fmtBuffer;

            return true;
        }
    }

    return false;
}
