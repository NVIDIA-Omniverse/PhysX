// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-READ-INSTANCER-001
 * @covers AC-2
 */

// clang-format off
#include <algorithm>
// clang-format on

#include <common/foundation/MatrixTools.h> // affineInverse -- these transforms carry scale
#include "tensors/cpu/CpuPointInstancerView.h"
#include "tensors/cpu/CpuSimulationView.h"

#include <PxPhysicsAPI.h>

#include <carb/logging/Log.h>

using namespace physx;

namespace omni
{
namespace physx
{
namespace tensors
{

CpuPointInstancerView::CpuPointInstancerView(CpuSimulationView* sim,
                                             const std::vector<PointInstancerEntry>& entries)
    : BasePointInstancerView(sim, entries)
{
}

bool CpuPointInstancerView::getInstancerColumnsOvStage(const InstancerColumn column,
                                                       void* const dst,
                                                       const PxU32* const offsets,
                                                       const PxU32 numInstancers) const
{
    // A failed check() is a decline: the view no longer describes the scene and the reader falls
    // back to the host path, so it is not logged. The two below are programming errors -- the
    // caller sized `offsets` from this very view -- and must be distinguishable from that decline.
    if (!check())
        return false;
    if (!dst || !offsets)
    {
        CARB_LOG_ERROR("%s: null destination or offsets", __FUNCTION__);
        return false;
    }
    if (numInstancers != mEntries.size())
    {
        CARB_LOG_ERROR("%s: called with %u instancers, view holds %zu", __FUNCTION__, numInstancers,
                       mEntries.size());
        return false;
    }

    // Orientation is the only 4-lane column; everything else is a 3-vector.
    const PxU32 lanes = column == InstancerColumn::eLocalOrientation ? 4u : 3u;

    float* const base = static_cast<float*>(dst);
    for (PxU32 i = 0; i < numInstancers; ++i)
    {
        const PointInstancerEntry& e = mEntries[i];
        float* const col = base + offsets[i];

        // Zeroed before the walk: the loop below SKIPS any slot whose body is gone or whose index is
        // out of range, and `dst` is caller-owned memory this view does not otherwise touch. Without
        // this, a skipped slot publishes whatever the caller's buffer happened to hold -- typically
        // the previous read's value for a different instance.
        std::fill_n(col, size_t(e.arrayLength) * lanes, 0.0f);

        for (const PointInstance& inst : e.instances)
        {
            if (!inst.body || inst.index >= e.arrayLength)
                continue;

            switch (column)
            {
            case InstancerColumn::eLocalPosition:
            case InstancerColumn::eLocalOrientation:
            {
                // Each column composes once and extracts only what it publishes: the rotation
                // extraction costs a sqrt plus three divides, so a position column calling the
                // both-values reframe would compute a quaternion purely to discard it. Asking for
                // the pair therefore composes twice, matching the kernel and the rigid pose columns.
                if (column == InstancerColumn::eLocalPosition)
                {
                    PxVec3 p;
                    reframePosition(inst.protoInverse, e.worldInverse, inst.body->getGlobalPose(), p);
                    float* o = col + size_t(inst.index) * 3u;
                    o[0] = p.x;
                    o[1] = p.y;
                    o[2] = p.z;
                }
                else
                {
                    PxVec3 p;
                    PxQuat q;
                    reframe(inst.protoInverse, e.worldInverse, inst.body->getGlobalPose(), p, q);
                    float* o = col + size_t(inst.index) * 4u;
                    o[0] = q.x;
                    o[1] = q.y;
                    o[2] = q.z;
                    o[3] = q.w;
                }
                break;
            }
            case InstancerColumn::eLinearVelocity:
            case InstancerColumn::eAngularVelocity:
            case InstancerColumn::eLinearAcceleration:
            case InstancerColumn::eAngularAcceleration:
            {
                // World frame, no reframe, and only the component asked for is queried. Acceleration
                // shares this arm with velocity because it differs only in which accessor is called.
                const bool wantAngular = (column == InstancerColumn::eAngularVelocity ||
                                          column == InstancerColumn::eAngularAcceleration);
                const bool wantAcceleration = (column == InstancerColumn::eLinearAcceleration ||
                                               column == InstancerColumn::eAngularAcceleration);
                const PxVec3 v =
                    wantAcceleration ?
                        (wantAngular ? inst.body->getAngularAcceleration() : inst.body->getLinearAcceleration()) :
                        (wantAngular ? inst.body->getAngularVelocity() : inst.body->getLinearVelocity());
                float* o = col + size_t(inst.index) * 3u;
                o[0] = v.x;
                o[1] = v.y;
                o[2] = v.z;
                break;
            }
            }
        }
    }
    return true;
}


// The write direction. Mirrors getInstancerColumnsOvStage line for line -- same validation, same
// per-instancer / per-instance walk, same offsets contract -- so the two cannot disagree about which
// slot is which.
bool CpuPointInstancerView::setInstancerColumnOvStage(const InstancerColumn column,
                                                      const PxU32 instancerIndex,
                                                      const void* const src,
                                                      const PxU32 arrayLength)
{
    // The same decline getInstancerColumnsOvStage opens with: a view that no longer describes the
    // scene declines here rather than dereferencing the stale bodies below. The checks after it are
    // caller errors, distinguishable from the decline.
    if (!check())
        return false;
    if (!src)
    {
        CARB_LOG_ERROR("%s: null source", __FUNCTION__);
        return false;
    }
    if (instancerIndex >= mEntries.size())
    {
        CARB_LOG_ERROR("%s: instancer %u of %zu", __FUNCTION__, instancerIndex, mEntries.size());
        return false;
    }
    {
        const PointInstancerEntry& e = mEntries[instancerIndex];
        // The caller sized its column from the array length this view reported. If they disagree the
        // view has been rebuilt underneath the group, and writing anyway would place instances at
        // indices the caller never meant.
        if (arrayLength != e.arrayLength)
        {
            CARB_LOG_ERROR("%s: column carries %u slots, instancer %u now has %u", __FUNCTION__, arrayLength,
                           instancerIndex, e.arrayLength);
            return false;
        }
        const float* const col = static_cast<const float*>(src);

        for (const PointInstance& inst : e.instances)
        {
            // The HOLE case, and the only place it is decided: a slot with no live body, or an index
            // past the array, is SKIPPED. There is nothing to write to. The read leaves the same
            // slot as the caller zero-filled it, so both directions agree that an index without a
            // body is not addressable.
            if (!inst.body || inst.index >= e.arrayLength)
                continue;

            switch (column)
            {
            case InstancerColumn::eLocalPosition:
            case InstancerColumn::eLocalOrientation:
            {
                // A pose write needs BOTH halves, and a session carries one attribute -- so the half
                // this column does not supply is read back from the body and preserved. Same
                // read-modify-write the rigid pose path performs, for the same reason, and it is why
                // the forward reframe is called here: the current world pose has to become a local
                // pose before the caller's half can be substituted into it.
                //
                // proto and instancerWorld are the FORWARD transforms, which nothing stores -- the
                // entry holds worldInverse and the instance holds protoInverse -- so both are
                // inverted back here.
                const ::physx::PxMat44d proto = omni::physx::affineInverse(inst.protoInverse);
                const ::physx::PxMat44d instancerWorld = omni::physx::affineInverse(e.worldInverse);

                PxVec3 curPos;
                PxQuat curRot;
                reframe(inst.protoInverse, e.worldInverse, inst.body->getGlobalPose(), curPos, curRot);

                if (column == InstancerColumn::eLocalPosition)
                {
                    const float* o = col + size_t(inst.index) * 3u;
                    curPos = PxVec3(o[0], o[1], o[2]);
                }
                else
                {
                    const float* o = col + size_t(inst.index) * 4u;
                    curRot = PxQuat(o[0], o[1], o[2], o[3]);
                }

                PxTransform world;
                unreframe(proto, instancerWorld, curPos, curRot, world);
                inst.body->setGlobalPose(world);
                break;
            }
            case InstancerColumn::eLinearVelocity:
            {
                // World frame, so no reframe: a velocity is not a pose and the instancer transform
                // does not apply to it. Same reason the read publishes these unreframed.
                const float* o = col + size_t(inst.index) * 3u;
                inst.body->setLinearVelocity(PxVec3(o[0], o[1], o[2]));
                break;
            }
            case InstancerColumn::eAngularVelocity:
            {
                const float* o = col + size_t(inst.index) * 3u;
                inst.body->setAngularVelocity(PxVec3(o[0], o[1], o[2]));
                break;
            }
            }
        }
    }
    return true;
}

} // namespace tensors
} // namespace physx
} // namespace omni
