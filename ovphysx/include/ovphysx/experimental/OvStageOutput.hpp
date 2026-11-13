// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-OVSTAGE-OUTPUT-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7
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

/** Optional application-owned reusable CPU/CUDA matrix buffers, one per
 * published prim set and device. Binds to the instance, stage pointer and
 * attachment on its first use. Destroy before the instance or stage. Serialize
 * access with simulation and stage edits. Call refresh() after topology changes
 * to release buffers for prim sets that are no longer published; refreshing
 * keeps the binding and does not permit reuse after reattachment.
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
    friend OVPHYSX_API OvStageOutputResult writeWorldTransformsToOvstage(ovphysx_instance_handle_t,
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
 * chooses outputOrdinal above the sealed write floor. After writing, the helper
 * computes the hierarchy at outputOrdinal with the CPU model, whatever the
 * runtime default. This sets the published prims' local transforms to match
 * their new world matrices, so a later parent edit moves them with their
 * parent. This helper does not step, seal or drain. Seal this output before the
 * next call; never drain physics output ordinals back into physics.
 *
 * Each matrix composes the physics position and orientation with the signed
 * world scale ovphysx stored when the body was created (OVPHYSX_ATTR_WORLD_SCALE),
 * read in the same physics read; no stage world matrix is read, so none has to
 * exist before the first call, and direct edits to the stage's world matrices do
 * not change the published scale. Neither does a runtime scale edit that physics
 * does not apply (character controllers, DirectGPU rigid bodies, articulation
 * links, kinematic bodies driven by time-sampled transforms): each publication
 * overwrites the matrix a hierarchy recomputation derived from the edit with one
 * at the creation scale, and its hierarchy computation folds that matrix into the
 * local transform. A character controller's orientation is its capsule
 * actor's, rotated so the capsule axis follows the up axis, while its scale keeps
 * the prim's authored axes, so its matrix is not the prim's. The stored scale is the
 * parser's decomposition of the world transform and does not preserve shear. For a
 * nondegenerate, shear-free world matrix, a negative linear determinant gives
 * three negative scale components; a positive one gives three positive
 * components. The physics rotation absorbs the remaining signs, so it need not
 * equal the authored rotation. Orientation writes must use the same physics
 * frame. On a DirectGPU scene the pose and scale columns are device-resident and
 * the matrices are composed on that device without a host copy.
 *
 * Failures return status and message. A stage other than the instance's attached
 * stage is rejected before stage access or cache binding. Earlier writes,
 * reported by matricesWritten, remain unsealed on a later write failure; there
 * is no rollback.
 */
OVPHYSX_API OvStageOutputResult writeWorldTransformsToOvstage(ovphysx_instance_handle_t instance,
                                                              ovstage_instance_t* stage,
                                                              ovstage_ordinal_t outputOrdinal,
                                                              OvStageOutputCache* cache = nullptr);

} // namespace utils
} // namespace ovphysx
