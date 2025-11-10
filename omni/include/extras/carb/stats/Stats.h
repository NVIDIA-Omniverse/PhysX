// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <carb/Defines.h>
#include <carb/Framework.h>

// clang-format off
#include "IStats.h"
// clang-format on

//-------------------------------------------------
// How to use IStats interface
//
// For IStats to work in your plugin, follow these steps:
//
// 1- Add CARB_ISTATS_GLOBALS() next to your CARB_PLUGIN_IMPL.
// 2- carb::stats::registerStatsInterfaceForClient() to register IStats at start up of your plugin,
//    or during carbOnPluginStartup() which is not always the best place.
// 3- Get the optional IStats interface with getStatsInterface().
// 4- Add new scopes and stats items via IStats.h APIs, such as getOrCreateScope() and getOrCreateStat().
//
//-------------------------------------------------

extern carb::stats::IStats* g_carbIStats;

#define CARB_ISTATS_GLOBALS() carb::stats::IStats* g_carbIStats = nullptr;

namespace carb
{
namespace stats
{

inline IStats* getStatsInterface()
{
    return g_carbIStats;
}

inline void registerStatsInterfaceForClient()
{
    g_carbIStats = getFramework()->tryAcquireInterface<IStats>();
}

inline void deregisterStatsInterfaceForClient()
{
    if (g_carbIStats)
    {
        g_carbIStats = nullptr;
    }
}
} // namespace stats
} // namespace carb
