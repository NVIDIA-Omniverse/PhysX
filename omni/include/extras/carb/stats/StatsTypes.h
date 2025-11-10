// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <carb/Types.h>

namespace gpu
{
namespace foundation
{
struct GpuFoundation;
}
} // namespace gpu

namespace carb
{
namespace stats
{

using StatId = void*;
using ScopeId = void*;

#define OMNI_MAX_SCOPE_NAME_LEN 128

/**
 * Defines flags for the scope
 */
typedef uint32_t ScopeFlags;
constexpr ScopeFlags kScopeFlagNone = 0; ///< Default flags.
constexpr ScopeFlags kScopeFlagRate = 1 << 0; ///< Interpreted as a rate, i.e., amount per time.
constexpr ScopeFlags kScopeFlagMemory = 1 << 1; ///< Interpreted as memory size, from byte to megabyte
constexpr ScopeFlags kScopeFlagHide = 1 << 2; ///< Scope should remain hidden or disabled, e.g. In UI.

/**
 * Defines the data type of statistics.
 */
enum class StatType : int32_t
{
    eUint64, ///< uin64_t data type counters.
    eDouble, ///< double data type counters.
};

union DoubleOrUint64
{
    uint64_t u64;
    double f64;
};

constexpr uint32_t kBytesPerGpuStat = 8;
constexpr uint32_t kInvalidGpuStatsBufferIndex = ~0u;

typedef uint32_t GpuStatsBufferFlags;
constexpr GpuStatsBufferFlags kGpuStatsBufferFlagNone = 0;
/// Reset stats in the GPU buffer to zero after completing a readback.
constexpr GpuStatsBufferFlags kGpuStatsBufferFlagResetOnReadback = 1 << 0;

/**
 * The description of a GPU stats buffer for gathering stats on the GPU.
 */
struct GpuStatsBufferDesc
{
    gpu::foundation::GpuFoundation* gpu;
    /// The initial number of stats the buffer can hold.
    uint32_t initialStatCapacity;
    /// The number of stat readbacks carb.stats will issue before assuming the oldest readback is complete.
    uint32_t maxReadbacksInFlight;
    GpuStatsBufferFlags flags;
};

/**
 * The description of a user-defined scope for the collection of the statistics.
 * Each scope can group multiple stats.
 */
struct ScopeDesc
{
    const char* scopeName; ///< A short and unique name used to group all the stats.
    const char* scopeDescription; ///< The description of this scope, e.g. displayed in UI for more information.
                                  ///< Maximum length: OMNI_MAX_SCOPE_NAME_LEN
    ScopeFlags flags; ///< Flags to modify scope behavior.
};

/**
 * The description of a user-defined statistics collector within a specific scope.
 */
struct StatDesc
{
    const char* statName; ///< A short and unique name within the specified scope, which collects info.
    const char* statDescription; ///< The description of the stats, e.g. displayed in UI for more information
    ScopeId scopeId; ///< The ID of the scope containing this stat.
    StatType type; ///< The data type for the Stats object.
};

/**
 * The description of each scope node.
 */
struct ScopeNode
{
    ScopeId scopeId; ///< The scopeId assigned to this node.
    ScopeFlags flags; ///< The scope flags.
    const char* scopeName; ///< The unique name of the scope.
    char scopeDescription[OMNI_MAX_SCOPE_NAME_LEN]; ///< The description of this scope.
};

/**
 * The description of each stats node.
 */
struct StatNode
{
    StatId statId; ///< The statId assigned to this node.
    StatType type; ///< The data type for this node.
    DoubleOrUint64 value; ///< The current double or uint64_t value, depending on the StatType.
    const char* statName; ///< The unique name of the stats within the scope.
    const char* statDescription; ///< The description of the stats.
    DoubleOrUint64 maxValue; ///< The max value attained by this stat. May be reset in the developer stats UI
};

} // namespace stats
} // namespace carb
