// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "StatsTypes.h"

namespace carb
{
namespace graphics
{
struct CommandList;
struct Descriptor;
}

namespace stats
{

struct IStats
{
    CARB_PLUGIN_INTERFACE("carb::stats::IStats", 2, 1)

    /**
     * Inserts a new scope for stats collection or returns an existing one if it is already added.
     *
     * @param scopeDesc A pointer ScopeDesc. Must be fully filled out.
     * @returns the ID of the new scope, or nullptr in case of failure.
     *
     * @remarks This method should not be called every frame as it incurs a non-trivial amount of overhead,
     * and it is thread safe.
     */
    ScopeId(CARB_ABI* getOrCreateScope)(const ScopeDesc& scopeDesc);

    /**
     * Inserts a new stat within a scope or returns an existing one if it is already added.
     *
     * @param statDesc A StatDesc describing the desired stat. Must be fully filled out.
     * @returns The ID of the new stat object, or nullptr in case of a failure.
     *
     * @remarks This method should not be called every frame as it incurs a non-trivial amount of overhead,
     * and it is thread safe.
     */
    StatId(CARB_ABI* getOrCreateStat)(const StatDesc& statDesc);

    /**
     * Creates a new stats scope that supports GPU stats.
     *
     * @param scopeDesc parameters describing the new scope.
     * @param bufferDesc parameters describing the GPU buffer to which shaders will write stats.
     * @returns The ID of the new scope, or nullptr if a failure occurred.
     *
     * @remarks It is an error to call this method more than once for a given scope name.
     *
     * GPU stats use 64b counters where supported but fall back silently to 32b otherwise.
     */
    ScopeId(CARB_ABI* createGpuStatsScope)(const ScopeDesc& scopeDesc, const GpuStatsBufferDesc& bufferDesc);

    /**
     * Releases the GPU stats buffer associated with the given scopeId.
     *
     * @param scopeId The GPU stats scope ID whose GPU stats buffer should be released.
     *
     * @remarks Must only be called once for a given GPU stats scope, as stat buffers are not refcounted. Leaves the
     * ScopeId alive for consistency with the way non-GPU-stats-enabled stats work.
     */
    void(CARB_ABI* releaseGpuStatsScope)(ScopeId scopeId);

    /**
     * Creates stats and allocates GPU buffer space for the given StatDescs.
     *
     * @param begin Iterator to the first StatDesc.
     * @param end Iterator to one-past the last StatDesc.
     * @returns The starting index of the stats block in the GPU stats buffer, or kInvalidGpuStatsBufferIndex on failure.
     * Multiply by carb::stats::kBytesPerGpuStat to get the byte offset.
     *
     * @remarks All StatDescs must have the same ScopeId, which must have been created with createGpuStatsScope.
     * Guaranteed to return 0 on the first call for the given scope. GPU stat buffer indices increment by 1 from `begin`
     * to `end`, so if 4 is returned then `*begin` gets index 4, `*++begin` gets index 5, etc.
     */
    uint32_t(CARB_ABI* createGpuStatsBlock)(const StatDesc* begin, const StatDesc* end);

    /**
     * Get a buffer view for the GPU stats buffer on the specified device.
     *
     * @param scopeId The GPU stats enabled scope for which to get a stats buffer view.
     * @param deviceIndex The index of the desired device.
     * @returns A descriptor which can be used to access the stats buffer on the specified GPU device.
     */
    carb::graphics::Descriptor*(CARB_ABI* getGpuStatsBufferView)(ScopeId scopeId, uint32_t deviceIndex);

    /**
     * Add commands to read GPU stats back to the CPU.
     *
     * @param scopeId The GPU stats enabled scope whose buffer should be read back.
     * @param cmdList The command list in which to perform the readback.
     * @param deviceIndex Index of the GPU device on which to perform the readback.
     *
     * @remarks This method and processOldestGpuStatsReadback must be called in coordination. See the remarks for that
     * method.
     */
    void(CARB_ABI* addGpuStatsReadback)(
        ScopeId scopeId, carb::graphics::CommandList* cmdList, uint32_t deviceIndex);

    /**
     * Processes a readback and updates CPU-facing stat counters **if** enough readbacks are in flight.
     *
     * @param scopeId The GPU stats enabled scope whose GPU stats readback should be processed.
     *
     * @remarks Does nothing unless maxReadbacksInFlight are active for each device, at which point it processes the
     * oldest readback for each device and marks it complete. For this to work correctly, addGpuStatsReadback must be
     * called once for each device for every one call to this method. The call order must be consistent; either
     * readbacks should always be added before processing or vice versa.
     *
     * The caller is responsible for ensuring the maxReadbacksInFlight-th readback is fully visible to the CPU before
     * calling this method.
     *
     * Ideally we'd add readbacks and process the oldest in the same method, but we need to process stats for all
     * devices simultaneously and some renderers can't readily provide command lists for all devices at once.
     */
    void(CARB_ABI* processOldestGpuStatsReadback)(ScopeId scopeId);

    /**
     * Retrieves the entire scopes added to the system as node structures, including the numbers of the added scopes.
     * It is thread safe.
     *
     * @param nodes      A pointer to an array of ScopeNode to get all the info of scopes. It can be nullptr,
     *                   if nodeCount is only needed.
     * @param nodeCount  The numbers of the added scopes. It can be nullptr.
     */
    void(CARB_ABI* enumerateScopeNodes)(ScopeNode* nodes, size_t* nodeCount);

    /**
     * Retrieves the entire stats added for the specified scopeId, including the numbers of the added stats.
     * It is thread safe.
     *
     * @param scopeId    The scope identifier to get the stats from.
     * @param nodes      A pointer to an array of StatNode to get all the info of stats. It can be nullptr,
     *                   if nodeCount is only needed.
     * @param nodeCount  The numbers of the added scopes. It can be nullptr.
     */
    void(CARB_ABI* enumerateStatNodes)(ScopeId scopeId, StatNode* nodes, size_t* nodeCount);

    /**
     * Atomically increases the uint64_t stat by 1.
     * The StatType must be Unit64.
     */
    void(CARB_ABI* incrementStatUint64)(StatId stat);

    /**
     * Atomically decreases the uint64_t stat by 1.
     * The StatType must be Unit64.
     */
    void(CARB_ABI* decrementStatUint64)(StatId stat);

    /**
     * Atomically increases the uint64_t stat by the value.
     * The StatType must be Unit64.
     * Return stat value before this operation.
     */
    uint64_t(CARB_ABI* addStatUint64)(StatId stat, uint64_t value);

    /**
     * Atomically decreases the uint64_t stat by the value.
     * The StatType must be Unit64.
     * Return stat value before this operation.
     */
    uint64_t(CARB_ABI* subtractStatUint64)(StatId stat, uint64_t value);

    /**
     * Atomically sets the uint64_t stat by the value.
     * The StatType must be Unit64.
     */
    void(CARB_ABI* setStatUint64)(StatId stat, uint64_t value);

    /**
     * Atomically sets the double stat by the value.
     * The StatType must be Double.
     */
    void(CARB_ABI* setStatDouble)(StatId stat, double value);

    /**
     * Returns the current ScopeNode description that contains ScopeDesc info initially created this scope or after
     * being modified at runtime. getOrCreateScope() may return an existing scope with different flags than what was
     * expected to be created with. This can be used to compare with the values.
     *
     * @param scopeId    The scope identifier to get the ScopeDesc from.
     * @scopeNode        A copy of ScopeDesc, or a desc filled with zero values if ScopeId is invalid.
     *
     * @Return true for a successful return of ScopeNode.
     */
    bool(CARB_ABI* getScopeNode)(ScopeId scopeId, ScopeNode& scopeNode);

    /**
     * Updates the current scopeDescription assigned to a scope, which shows up in the stats UI.
     *
     * Note: Calling this function won't update UI per frame, unless another scope change such as
     * setScopeVisibility() is made.
     *
     * @param scopeId           The scope identifier to update the scopeDescription for.
     * @param scopeDescription  The string for the description of ScopeId.
     */
    void(CARB_ABI* setScopeDescription)(ScopeId scopeId, const char* scopeDescription);

    /**
     * Updates the visibility flag of a scope via kScopeFlagHide. Hidden scopes will not show up in the stats UI.
     * You can check for the visibility status via getScopeNode() and kScopeFlagHide flag.
     *
     * @param scopeId    The scope identifier to update the visibility flag for.
     * @param showScope  The boolean to show it hide the scope.
     */
    void(CARB_ABI* setScopeVisibility)(ScopeId scopeId, bool showScope);

    /**
     * Return stat values for uint64 and double respectively
    */
    uint64_t(CARB_ABI* getStatUint64)(StatId stat);
    double(CARB_ABI* getStatDouble)(StatId stat);
  

};

} // namespace stats
} // namespace carb
