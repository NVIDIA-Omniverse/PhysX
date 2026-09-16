// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @implements REQ-READ-INSTANCER-001
 * @covers AC-1, AC-9
 */

#include "tensors/CommonTypes.h"
#include "tensors/InstancerReframe.h"
#include "tensors/base/BaseSimulationData.h"

#include <vector>

namespace omni
{
namespace physx
{
namespace tensors
{
class BaseSimulationView;

// Deliberately not an ISimulationView-style public interface: the ovstage read is the only consumer
// and it holds the concrete type. REQ-READ-INSTANCER-001 records the shape an interface would take.
class BasePointInstancerView
{
public:
    BasePointInstancerView(BaseSimulationView* sim, const std::vector<PointInstancerEntry>& entries);
    // Virtual: release() does `delete this` through the BasePointInstancerView* BaseSimulationView holds.
    virtual ~BasePointInstancerView();

    bool check() const;
    void release();

    void _onParentRelease();

    // ----------------------------------------------------------------------------------------------
    // ovstage column reads.
    //
    // ONE call fills EVERY instancer's column for an attribute: per-prim work, not the gather, is what
    // a read pays for, so a per-instancer entry point would cost one call per group.
    //
    // The destination is one buffer the caller sub-allocates, and `offsets` gives each instancer's
    // start in FLOATS -- not in "elements", which in this codebase means the dtype.lanes tuple and
    // would put a reader off by `comp`. Getting that wrong scatters into the NEXT instancer's
    // sub-array with nothing to notice, the same hazard the comp-vs-kernel-stride check in
    // buildInstancerBackendGroups guards.
    //
    // One offsets array per COLUMN, not one shared across them: a column's sub-array is sized
    // (maxIndex + 1) * comp floats, so the starts differ between a 3-wide and a 4-wide attribute.
    // The caller's per-group tensors point into the column's region at these offsets.
    enum class InstancerColumn
    {
        eLocalPosition,   // 3 floats per slot, instancer frame
        eLocalOrientation, // 4 floats per slot, xyzw, instancer frame
        eLinearVelocity,  // 3 floats per slot, world frame
        eAngularVelocity, // 3 floats per slot, world frame
        // World frame like velocity, and shares its plumbing: same device scratch (mRd*VelAccDev),
        // same gather kernel, only the PxRigidDynamicGPUAPIReadType differs. PhysX returns ZERO unless
        // the scene carries PxSceneFlag::eENABLE_BODY_ACCELERATIONS, which PhysXScene sets on every
        // scene it creates -- a silent zero, not an error, if that changes.
        eLinearAcceleration,  // 3 floats per slot, world frame
        eAngularAcceleration, // 3 floats per slot, world frame
    };

    // `dst` is device memory on the GPU view and host memory on the CPU one, matching where that
    // view's columns live. Slots with no instance are left as the caller zero-filled them.
    virtual bool getInstancerColumnsOvStage(InstancerColumn column,
                                            void* dst,
                                            const ::physx::PxU32* offsets,
                                            ::physx::PxU32 numInstancers) const = 0;

    // The write direction (ADR-0012). ONE instancer per call, unlike the bulk getter
    // above, and that asymmetry is deliberate: a read fills every column in one pass, while a write
    // hands each instancer's array to the caller as its own group and commits them INDEPENDENTLY and
    // in any order. A bulk setter would have no moment at which all of them are filled.
    //
    // `src` is `arrayLength * comp` floats -- device memory on the GPU view, host on the CPU one --
    // placed by instance index, exactly as the read publishes it.
    //
    // HOLES ARE SKIPPED. A slot with no live instance has nothing to write to, so its value is
    // dropped rather than erroring or spawning. That is the counterpart of the read leaving such a
    // slot as the caller zero-filled it -- both directions agree an index without a body is not
    // addressable -- and it is stated because "the value I wrote did nothing" is otherwise
    // indistinguishable from a bug.
    //
    // A SCALED INSTANCER round-trips only up to its scale, for eLocalPosition / eLocalOrientation:
    // the local pair carries position and rotation, USD keeps `scales` as its own array, and no
    // inverse can restore what the forward direction discarded. See TestInstancerReframe.
    virtual bool setInstancerColumnOvStage(InstancerColumn column,
                                           ::physx::PxU32 instancerIndex,
                                           const void* src,
                                           ::physx::PxU32 arrayLength) = 0;

    // How many instancers this view holds, so a caller can bound `instancerIndex`.
    ::physx::PxU32 getInstancerCount() const
    {
        return static_cast<::physx::PxU32>(mEntries.size());
    }

    // The instancer's own world transform, inverted -- the frame every local pose is expressed in.
    //
    // Pushed in per read rather than cached behind the object-lifetime epoch, because it is a USD
    // stage query and not PhysX state: MOVING an instancer changes it while creating and destroying
    // nothing, so the epoch never bumps and a cached value would go quietly stale. The reader
    // resolves it each read and sets it here before reading any column.
    virtual bool setWorldInverses(const ::physx::PxMat44d* inverses, ::physx::PxU32 numInstancers);

    // Valid for the view's lifetime, with the instance lists fixed at construction. `worldInverse` is
    // NOT -- setWorldInverses rewrites it, and the reader calls that on every read by contract, so it is
    // the one field of an entry a caller must not cache.
    const std::vector<PointInstancerEntry>& getEntries() const
    {
        return mEntries;
    }

    // ----------------------------------------------------------------------------------------------
    // Static reframe utilities.
    // ----------------------------------------------------------------------------------------------


    // Both reframes below are thin adapters over InstancerReframe.h -- the SAME math the device kernel
    // runs, so no two paths can answer differently for the same instance. That header carries the
    // reasoning for the double precision and for matching Gf's rotation extraction exactly. Both take the
    // prototype matrix rather than a PointInstance because the reader's host fallback calls them with the
    // matrix off InternalActor and no instance struct to hand.


    // The WRITE direction: an instancer-local pose back to a world one (ADR-0012). Takes the
    // FORWARD transforms, where the reframes below take inverses -- and note neither forward matrix
    // is stored anywhere, so a caller inverts what it has.
    //
    // A single entry point rather than a position/orientation pair, unlike the read: the write
    // always has both halves of a pose to apply (PxTransform takes both), so there is no half to
    // save work by skipping.
    static void unreframe(const ::physx::PxMat44d& proto,
                          const ::physx::PxMat44d& instancerWorld,
                          const ::physx::PxVec3& localPos,
                          const ::physx::PxQuat& localRot,
                          ::physx::PxTransform& outWorld);

    // Position only: the composition without the rotation extraction, which costs a double sqrt and three
    // double divides per instance and yields a quaternion the host column reads would throw away.
    static void reframePosition(const ::physx::PxMat44d& protoInverse,
                                const ::physx::PxMat44d& instancerWorldInverse,
                                const ::physx::PxTransform& worldPose,
                                ::physx::PxVec3& outPos);

    // Position AND rotation, for the callers that publish both.
    static void reframe(const ::physx::PxMat44d& protoInverse,
                        const ::physx::PxMat44d& instancerWorldInverse,
                        const ::physx::PxTransform& worldPose,
                        ::physx::PxVec3& outPos,
                        ::physx::PxQuat& outRot);

protected:
    BaseSimulationView* mSim = nullptr;
    BaseSimulationDataPtr mSimData;

    // Set by setWorldInverses: true when that call changed at least one entry. Subclasses that
    // mirror the inverses somewhere expensive gate on this rather than on being called.
    bool mWorldInversesChanged = true;

    std::vector<PointInstancerEntry> mEntries;
};

} // namespace tensors
} // namespace physx
} // namespace omni
