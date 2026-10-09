// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-OVSTAGE-OUTPUT-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6
 */

#pragma once

#include <ovphysx/ovphysx.h>

#include <cstddef>
#include <memory>
#include <string>

namespace ovphysx
{
namespace utils
{

struct OvStageOutputResult
{
    ovphysx_api_status_t status = OVPHYSX_API_SUCCESS;
    size_t matricesWritten = 0;
    size_t instancerGroupsSkipped = 0;
    std::string message;

    bool ok() const
    {
        return status == OVPHYSX_API_SUCCESS;
    }
};

/** Optional application-owned scale snapshots and reusable CPU/CUDA buffers.
 * Binds to the instance, stage pointer and attachment on its first use.
 * Destroy before the instance or stage. Serialize access with simulation and
 * stage edits. Call refresh() after authored transforms or topology change;
 * refreshing keeps the binding and does not permit reuse after reattachment.
 */
class OvStageOutputCache
{
public:
    OVPHYSX_API OvStageOutputCache();
    OVPHYSX_API ~OvStageOutputCache();
    OvStageOutputCache(const OvStageOutputCache&) = delete;
    OvStageOutputCache& operator=(const OvStageOutputCache&) = delete;

    OVPHYSX_API void refresh();

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
    friend OVPHYSX_API OvStageOutputResult writeWorldTransformsToOvstage(ovphysx_handle_t,
                                                                         ovstage_instance_t*,
                                                                         ovstage_ordinal_t,
                                                                         OvStageOutputCache*);
};

/** Publish fixed rigid-body and articulation-link poses to omni:fabric:worldMatrix.
 * Supports CPU and CUDA columns. CUDA data must be addressable from the primary
 * context of its DLPack device ordinal, where the helper consumes it. The helper
 * uses CUDA without accessing the simulation's CUDA manager and restores the
 * calling thread's prior context.
 * Point-instancer array groups are skipped and counted in the result.
 *
 * The caller supplies the attached stage, waits for its simulation step, and
 * chooses outputOrdinal above the sealed write floor. Compute and seal current
 * world matrices before first use (including the hierarchy for nested bodies).
 * This helper does not step, seal, drain, update local transforms, or propagate
 * changes to descendants. Seal this output before the next call; never drain
 * physics output ordinals back into physics. A later hierarchy recomputation
 * can replace the written matrices with values derived from local transforms.
 *
 * With no cache, derive signed scale from the current sealed world matrices on
 * each call. A cache retains owned scales and buffers, never borrowed stage
 * views. Matrix decomposition follows the parser and does not preserve shear.
 * ovstage supplies CPU world-matrix reads for scale capture, including for
 * matrices authored on CUDA. Cached CUDA frames use device poses and scales.
 *
 * Failures return status and message. A stage other than the instance's attached
 * stage is rejected before stage access or cache binding. Earlier writes,
 * reported by matricesWritten, remain unsealed on a later write failure; there
 * is no rollback.
 */
OVPHYSX_API OvStageOutputResult writeWorldTransformsToOvstage(ovphysx_handle_t instance,
                                                              ovstage_instance_t* stage,
                                                              ovstage_ordinal_t outputOrdinal,
                                                              OvStageOutputCache* cache = nullptr);

} // namespace utils
} // namespace ovphysx
