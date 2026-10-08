// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-READ-INSTANCER-001
 * @covers AC-1, AC-2, AC-5
 */

// clang-format off
// clang-format on

#include "tensors/base/BasePointInstancerView.h"
#include "tensors/base/BaseSimulationView.h"

#include "tensors/CommonTypes.h"

#include <PxPhysicsAPI.h>

#include <carb/logging/Log.h>

#include <cstring>


using namespace physx;

namespace omni
{
namespace physx
{
namespace tensors
{

BasePointInstancerView::BasePointInstancerView(BaseSimulationView* sim,
                                               const std::vector<PointInstancerEntry>& entries)
    : mSim(sim), mEntries(entries)
{
    if (!mSim)
        return;

    // Hold the simulation data so what this view reads through cannot be deleted under it.
    mSimData = mSim->getBaseSimulationData();

    for (const PointInstancerEntry& e : mEntries)
    {
        // Registered like any other body this view reads, so the parent notices when one dies.
        for (const PointInstance& inst : e.instances)
            if (inst.body)
                mSim->rigidBodies.insert(inst.body);
    }
}

BasePointInstancerView::~BasePointInstancerView()
{
    // Deregister so release() on a REGISTERED view cannot leave the parent's vector holding a freed
    // pointer for ~BaseSimulationView to walk. Safe alongside releaseChildViews, which pops before
    // destroying: the find below simply misses.
    if (mSim)
        mSim->_onChildRelease(this);
}

void BasePointInstancerView::_onParentRelease()
{
    mSim = nullptr;
}

void BasePointInstancerView::release()
{
    delete this;
}

// Unlike its peers, this view does not re-resolve entries through g_physx: entries arrive already
// resolved from the ovstage reader (no path to re-resolve), and the reader drops the whole view when
// the object-lifetime epoch or the backend generation moves. Only parent liveness is left to check.
bool BasePointInstancerView::check() const
{
    return mSim != nullptr && mSimData != nullptr;
}

bool BasePointInstancerView::setWorldInverses(const ::physx::PxMat44d* const inverses, const PxU32 numInstancers)
{
    if (!inverses || numInstancers != mEntries.size())
        return false;
    // Records whether anything actually MOVED, not merely that the reader called again: the reader
    // re-resolves these every read, so keying an upload on "was I called" costs a blocking
    // host-to-device copy per read on a stage whose instancers never move.
    //
    // A bitwise compare can only err in the safe direction -- it may report a change that did not
    // happen, never miss one that did -- so a -0.0 for a +0.0 costs a redundant upload, not a stale
    // one.
    mWorldInversesChanged = false;
    for (PxU32 i = 0; i < numInstancers; ++i)
    {
        // sizeof the MEMBER, not the type: worldInverse is declared in CommonTypes.h, and a narrower
        // declaration there would silently make this read past it.
        if (std::memcmp(&mEntries[i].worldInverse, &inverses[i], sizeof(mEntries[i].worldInverse)) != 0)
        {
            mEntries[i].worldInverse = inverses[i];
            mWorldInversesChanged = true;
        }
    }
    return true;
}

// Widen the float pose to the double precision the shared composition math runs in.
static void toSharedInputs(const PxTransform& worldPose,
                           double (&outWorldQuat)[4],
                           double (&outWorldPos)[3])
{
    outWorldQuat[0] = double(worldPose.q.x);
    outWorldQuat[1] = double(worldPose.q.y);
    outWorldQuat[2] = double(worldPose.q.z);
    outWorldQuat[3] = double(worldPose.q.w);
    outWorldPos[0] = double(worldPose.p.x);
    outWorldPos[1] = double(worldPose.p.y);
    outWorldPos[2] = double(worldPose.p.z);
}

void BasePointInstancerView::unreframe(const ::physx::PxMat44d& proto,
                                       const ::physx::PxMat44d& instancerWorld,
                                       const PxVec3& localPos,
                                       const PxQuat& localRot,
                                       PxTransform& outWorld)
{
    const double localQuat[4] = { double(localRot.x), double(localRot.y), double(localRot.z),
                                  double(localRot.w) };
    const double localP[3] = { double(localPos.x), double(localPos.y), double(localPos.z) };
    double worldPos[3];
    double worldQuat[4];
    // instancerUnreframe rather than composing here, so this stays the entry point
    // TestInstancerReframe's round-trip case exercises.
    instancerUnreframe(proto, instancerWorld, localQuat, localP, worldPos, worldQuat);
    outWorld = PxTransform(PxVec3(float(worldPos[0]), float(worldPos[1]), float(worldPos[2])),
                           PxQuat(float(worldQuat[0]), float(worldQuat[1]), float(worldQuat[2]),
                                  float(worldQuat[3])));
}

void BasePointInstancerView::reframePosition(const ::physx::PxMat44d& protoInverse,
                                             const ::physx::PxMat44d& instancerWorldInverse,
                                             const PxTransform& worldPose,
                                             PxVec3& outPos)
{
    double worldQuat[4];
    double worldPos[3];
    toSharedInputs(worldPose, worldQuat, worldPos);

    // Stops at the composition, exactly as the kernel's position branch does.
    InstancerAffine local;
    instancerComposeLocal(protoInverse, instancerWorldInverse, worldQuat, worldPos, local);
    outPos = PxVec3(float(local.column3.x), float(local.column3.y), float(local.column3.z));
}

void BasePointInstancerView::reframe(const ::physx::PxMat44d& protoInverse,
                                     const ::physx::PxMat44d& instancerWorldInverse,
                                     const PxTransform& worldPose,
                                     PxVec3& outPos,
                                     PxQuat& outRot)
{
    double worldQuat[4];
    double worldPos[3];
    toSharedInputs(worldPose, worldQuat, worldPos);

    // Call instancerReframe rather than inlining an equivalent, so this stays the entry point
    // TestInstancerReframe exercises.
    double localPos[3];
    double localQuat[4];
    instancerReframe(protoInverse, instancerWorldInverse, worldQuat, worldPos, localPos, localQuat);

    outPos = PxVec3(float(localPos[0]), float(localPos[1]), float(localPos[2]));
    outRot = PxQuat(float(localQuat[0]), float(localQuat[1]), float(localQuat[2]), float(localQuat[3]));
}

} // namespace tensors
} // namespace physx
} // namespace omni
