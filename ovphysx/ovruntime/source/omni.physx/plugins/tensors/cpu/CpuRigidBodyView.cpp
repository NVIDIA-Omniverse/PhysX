// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-READ-CORE-001
 * @covers AC-1
 *
 * @implements REQ-READ-ATTRS-001
 * @covers AC-1, AC-3, AC-5
 *
 * @implements REQ-TENSOR-INDEX-001
 * @covers AC-1 AC-2 AC-3
 */

#include "tensors/cpu/CpuRigidBodyView.h"
#include "tensors/cpu/CpuSimulationView.h"

#include "tensors/GlobalsAreBad.h"
#include "tensors/CommonTypes.h"

#include <carb/logging/Log.h>

#include <PxPhysicsAPI.h>
#include <omni/physics/tensors/TensorUtils.h>

using omni::physics::tensors::checkRecordIndices;
using omni::physics::tensors::checkTensorDevice;
using omni::physics::tensors::checkTensorFloat32;
using omni::physics::tensors::checkTensorInt32;
using omni::physics::tensors::checkTensorSizeExact;
using omni::physics::tensors::checkTensorSizeMinimum;
using omni::physics::tensors::getTensorTotalSize;

using namespace physx;

namespace omni
{
namespace physx
{
namespace tensors
{

CpuRigidBodyView::CpuRigidBodyView(CpuSimulationView* sim, const std::vector<RigidBodyEntry>& entries)
    : BaseRigidBodyView(sim, entries)
{
    uint32_t numBodies = uint32_t(mEntries.size());

    // for bodies that are root articulation links, we use the articulation cache to set transforms and velocities
    mArticulations.resize(numBodies);
    mArticulationCaches.resize(numBodies);
    for (PxU32 i = 0; i < numBodies; i++)
    {
        const RigidBodyEntry& entry = mEntries[i];
        if (entry.type == RigidBodyType::eArticulationLink)
        {
            const PxArticulationLink* link = static_cast<const PxArticulationLink*>(entry.body);
            if (link->getLinkIndex() == 0)
            {
                PxArticulationReducedCoordinate& arti =
                    static_cast<PxArticulationReducedCoordinate&>(link->getArticulation());
                mArticulations[i] = &arti;
                mArticulationCaches[i] = arti.createCache();
            }
        }
    }

    mCpuSimData = sim->getCpuSimulationData();
}

CpuRigidBodyView::~CpuRigidBodyView()
{
    // Releasing an articulation does not free caches created from it, so the view owns these.
    // Skipped once the PhysX plugin unloads: release() frees through the foundation allocator.
    if (g_physx)
    {
        for (PxArticulationCache* cache : mArticulationCaches)
        {
            if (cache)
            {
                cache->release();
            }
        }
    }
    mArticulationCaches.clear();
}

bool CpuRigidBodyView::getTransforms(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "transform", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "transform", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * 7u, "transform", __FUNCTION__))
    {
        return false;
    }

    float* dst = static_cast<float*>(dstTensor->data);
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        PxTransform pose = mEntries[i].body->getGlobalPose();
        Subspace* subspace = mEntries[i].subspace;
        if (subspace)
        {
            pose.p.x -= subspace->origin.x;
            pose.p.y -= subspace->origin.y;
            pose.p.z -= subspace->origin.z;
        }
        *dst++ = pose.p.x;
        *dst++ = pose.p.y;
        *dst++ = pose.p.z;
        *dst++ = pose.q.x;
        *dst++ = pose.q.y;
        *dst++ = pose.q.z;
        *dst++ = pose.q.w;
    }

    return true;
}


bool CpuRigidBodyView::getPositionsOvStage(const TensorDesc* dstTensor,
                                          const PxU32* outRecordIdx,
                                          const PxU32 numOutputs,
                                          uint64_t) const
{
    return gatherPoseColumnOvStage(/*wantOrientation=*/false, outRecordIdx, numOutputs, dstTensor);
}

bool CpuRigidBodyView::getOrientationsOvStage(const TensorDesc* dstTensor,
                                              const PxU32* outRecordIdx,
                                              const PxU32 numOutputs,
                                              uint64_t) const
{
    return gatherPoseColumnOvStage(/*wantOrientation=*/true, outRecordIdx, numOutputs, dstTensor);
}

bool CpuRigidBodyView::getLinearVelocitiesOvStage(const TensorDesc* dstTensor,
                                                  const PxU32* outRecordIdx,
                                                  const PxU32 numOutputs,
                                                  uint64_t) const
{
    return gatherVelAccColumnOvStage(/*wantAngular=*/false, /*wantAcceleration=*/false, outRecordIdx, numOutputs,
                                     dstTensor);
}

bool CpuRigidBodyView::getAngularVelocitiesOvStage(const TensorDesc* dstTensor,
                                                   const PxU32* outRecordIdx,
                                                   const PxU32 numOutputs,
                                                   uint64_t) const
{
    return gatherVelAccColumnOvStage(/*wantAngular=*/true, /*wantAcceleration=*/false, outRecordIdx, numOutputs,
                                     dstTensor);
}

bool CpuRigidBodyView::getLinearAccelerationsOvStage(const TensorDesc* dstTensor,
                                                     const PxU32* outRecordIdx,
                                                     const PxU32 numOutputs,
                                                     uint64_t) const
{
    return gatherVelAccColumnOvStage(/*wantAngular=*/false, /*wantAcceleration=*/true, outRecordIdx, numOutputs,
                                     dstTensor);
}

bool CpuRigidBodyView::getAngularAccelerationsOvStage(const TensorDesc* dstTensor,
                                                      const PxU32* outRecordIdx,
                                                      const PxU32 numOutputs,
                                                      uint64_t) const
{
    return gatherVelAccColumnOvStage(/*wantAngular=*/true, /*wantAcceleration=*/true, outRecordIdx, numOutputs,
                                     dstTensor);
}

bool CpuRigidBodyView::setPositionsOvStage(const TensorDesc* srcTensor,
                                           const PxU32* outRecordIdx,
                                           const PxU32 numOutputs,
                                           uint64_t)
{
    return scatterPoseColumnOvStage(/*wantOrientation=*/false, outRecordIdx, numOutputs, srcTensor);
}

bool CpuRigidBodyView::setOrientationsOvStage(const TensorDesc* srcTensor,
                                              const PxU32* outRecordIdx,
                                              const PxU32 numOutputs,
                                              uint64_t)
{
    return scatterPoseColumnOvStage(/*wantOrientation=*/true, outRecordIdx, numOutputs, srcTensor);
}

bool CpuRigidBodyView::setLinearVelocitiesOvStage(const TensorDesc* srcTensor,
                                                  const PxU32* outRecordIdx,
                                                  const PxU32 numOutputs,
                                                  uint64_t)
{
    return scatterVelocityColumnOvStage(/*wantAngular=*/false, outRecordIdx, numOutputs, srcTensor);
}

bool CpuRigidBodyView::setAngularVelocitiesOvStage(const TensorDesc* srcTensor,
                                                   const PxU32* outRecordIdx,
                                                   const PxU32 numOutputs,
                                                   uint64_t)
{
    return scatterVelocityColumnOvStage(/*wantAngular=*/true, outRecordIdx, numOutputs, srcTensor);
}

// The scatter counterpart of gatherPoseColumnOvStage. Deliberately shaped like it: same record
// indirection, same validation, the subspace origin ADDED where the read subtracts it, so a column
// read, edited and written back round-trips to the value it started from.
bool CpuRigidBodyView::scatterPoseColumnOvStage(const bool wantOrientation,
                                                const PxU32* outRecordIdx,
                                                const PxU32 numOutputs,
                                                const TensorDesc* srcTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    if (!srcTensor || !srcTensor->data)
        return false;

    const char* what = wantOrientation ? "orientation" : "position";
    const PxU32 comp = wantOrientation ? 4u : 3u;
    if (!checkTensorDevice(*srcTensor, -1, what, __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, what, __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, numOutputs * comp, what, __FUNCTION__) ||
        !checkRecordIndices(outRecordIdx, numOutputs, mEntries.size(), what, __FUNCTION__))
    {
        return false;
    }

    const float* src = static_cast<const float*>(srcTensor->data);
    for (PxU32 i = 0; i < numOutputs; i++)
    {
        const PxU32 recIdx = outRecordIdx ? outRecordIdx[i] : i;
        const RigidBodyEntry& e = mEntries[recIdx];
        // An articulation link's pose is derived from the root and the joint state; setGlobalPose on
        // one is not a supported operation. Skipped rather than attempted, matching the GPU path,
        // where PxArticulationGPUAPIWriteType has no eLINK_GLOBAL_POSE at all.
        if (e.type != RigidBodyType::eRigidDynamic)
        {
            src += comp;
            continue;
        }

        // Read-modify-write: setGlobalPose takes a whole transform and a session writes one
        // attribute, so the component not being written has to come from the body itself.
        PxTransform pose = e.body->getGlobalPose();
        if (wantOrientation)
        {
            pose.q = PxQuat(src[0], src[1], src[2], src[3]);
        }
        else
        {
            const Subspace* subspace = e.subspace;
            const PxVec3 origin =
                subspace ? PxVec3(subspace->origin.x, subspace->origin.y, subspace->origin.z) : PxVec3(0.0f);
            pose.p = PxVec3(src[0] + origin.x, src[1] + origin.y, src[2] + origin.z);
        }
        e.body->setGlobalPose(pose);
        src += comp;
    }
    return true;
}

bool CpuRigidBodyView::setWrenchesOvStage(const TensorDesc* srcTensor,
                                         const PxU32* outRecordIdx,
                                         const PxU32 numOutputs,
                                         const uint64_t /*rowsToken*/)
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    if (!srcTensor || !srcTensor->data)
        return false;
    if (numOutputs == 0)
        return true;
    if (!checkTensorDevice(*srcTensor, -1, "wrench", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "wrench", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, numOutputs * 9u, "wrench", __FUNCTION__) ||
        !checkRecordIndices(outRecordIdx, numOutputs, mEntries.size(), "wrench", __FUNCTION__))
    {
        return false;
    }

    const float* src = static_cast<const float*>(srcTensor->data);
    for (PxU32 i = 0; i < numOutputs; i++)
    {
        const PxU32 recIdx = outRecordIdx ? outRecordIdx[i] : i;
        PxRigidBody* body = mEntries[recIdx].body;
        if (!body)
        {
            src += 9;
            continue;
        }
        // Applying force to a kinematic actor crashes PhysX (the binding force path guards this too);
        // skip the row as a no-op rather than call addForceAtPos/addTorque on it.
        if (body->getRigidBodyFlags().isSet(PxRigidBodyFlag::eKINEMATIC))
        {
            src += 9;
            continue;
        }
        const PxVec3 force(src[0], src[1], src[2]);
        const PxVec3 torque(src[3], src[4], src[5]);
        const PxVec3 point(src[6], src[7], src[8]);

        // addForceAtPos does the (point - comWorld) x force conversion itself, in WORLD space, which
        // is why this path spells the conversion out only on the GPU side: there is no host helper
        // there. Using PhysX's own helper here is deliberate -- two hand-written copies of the same
        // cross product is exactly the drift the shared-derivation rule elsewhere in this API exists
        // to prevent, and this one cannot be shared because the device needs it in a kernel.
        ::physx::PxRigidBodyExt::addForceAtPos(*body, force, point, PxForceMode::eFORCE);
        // The torque half carries no application point by construction: a couple is
        // position-independent, so it is added as-is rather than through the helper.
        if (!torque.isZero())
            body->addTorque(torque, PxForceMode::eFORCE);
        src += 9;
    }
    return true;
}

bool CpuRigidBodyView::setForcesOvStage(const TensorDesc* srcTensor,
                                       const PxU32* outRecordIdx,
                                       const PxU32 numOutputs,
                                       const uint64_t /*rowsToken*/)
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    if (!srcTensor || !srcTensor->data)
        return false;
    if (numOutputs == 0)
        return true;
    if (!checkTensorDevice(*srcTensor, -1, "force", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "force", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, numOutputs * 3u, "force", __FUNCTION__) ||
        !checkRecordIndices(outRecordIdx, numOutputs, mEntries.size(), "force", __FUNCTION__))
    {
        return false;
    }

    const float* src = static_cast<const float*>(srcTensor->data);
    for (PxU32 i = 0; i < numOutputs; i++)
    {
        const PxU32 recIdx = outRecordIdx ? outRecordIdx[i] : i;
        PxRigidBody* body = mEntries[recIdx].body;
        // No RigidBodyType gate here, unlike the velocity scatter: addForce is a PxRigidBody method,
        // so an articulation LINK takes it. That is the whole difference between this attribute and
        // linearVelocity, and it is why `force` is marked linkWritable and velocity is not.
        if (!body)
        {
            src += 3;
            continue;
        }
        // Applying force to a kinematic actor crashes PhysX (the binding force path guards this too);
        // a link is never kinematic, so this only skips kinematic rigid dynamics.
        if (body->getRigidBodyFlags().isSet(PxRigidBodyFlag::eKINEMATIC))
        {
            src += 3;
            continue;
        }
        // addForce, not a setter: PhysX accumulates within the step and clears at the end of it.
        // eFORCE mode means the value is a force, applied at the centre of mass.
        body->addForce(PxVec3(src[0], src[1], src[2]), PxForceMode::eFORCE);
        src += 3;
    }
    return true;
}

// As above for a velocity component. No read first: linear and angular have independent setters, so
// neither pays for the other.
bool CpuRigidBodyView::scatterVelocityColumnOvStage(const bool wantAngular,
                                                    const PxU32* outRecordIdx,
                                                    const PxU32 numOutputs,
                                                    const TensorDesc* srcTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    if (!srcTensor || !srcTensor->data)
        return false;

    const char* what = wantAngular ? "angular velocity" : "linear velocity";
    if (!checkTensorDevice(*srcTensor, -1, what, __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, what, __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, numOutputs * 3u, what, __FUNCTION__) ||
        !checkRecordIndices(outRecordIdx, numOutputs, mEntries.size(), what, __FUNCTION__))
    {
        return false;
    }

    const float* src = static_cast<const float*>(srcTensor->data);
    for (PxU32 i = 0; i < numOutputs; i++)
    {
        const PxU32 recIdx = outRecordIdx ? outRecordIdx[i] : i;
        const RigidBodyEntry& e = mEntries[recIdx];
        // The setters live on PxRigidDynamic, not PxRigidBody: an articulation link's velocity
        // follows from the root and the joint state and is not settable on the link. Gated on the
        // entry's recorded type rather than a dynamic_cast, matching setVelocities below, and
        // skipped rather than attempted -- the same rows the GPU path leaves unflagged.
        if (e.type != RigidBodyType::eRigidDynamic)
        {
            src += 3;
            continue;
        }
        // Velocity is frame-free, so unlike position there is no origin to reframe.
        const PxVec3 v(src[0], src[1], src[2]);
        PxRigidDynamic* rd = static_cast<PxRigidDynamic*>(e.body);
        if (wantAngular)
            rd->setAngularVelocity(v);
        else
            rd->setLinearVelocity(v);
        src += 3;
    }
    return true;
}

bool CpuRigidBodyView::gatherPoseColumnOvStage(const bool wantOrientation,
                                               const PxU32* outRecordIdx,
                                               const PxU32 numOutputs,
                                               const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    if (!dstTensor || !dstTensor->data)
        return false;

    const char* what = wantOrientation ? "orientation" : "position";
    const PxU32 comp = wantOrientation ? 4u : 3u;
    if (!checkTensorDevice(*dstTensor, -1, what, __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, what, __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, numOutputs * comp, what, __FUNCTION__) ||
        !checkRecordIndices(outRecordIdx, numOutputs, mEntries.size(), what, __FUNCTION__))
    {
        return false;
    }

    float* dst = static_cast<float*>(dstTensor->data);
    for (PxU32 i = 0; i < numOutputs; i++)
    {
        const PxU32 recIdx = outRecordIdx ? outRecordIdx[i] : i;
        const PxTransform pose = mEntries[recIdx].body->getGlobalPose();
        if (wantOrientation)
        {
            *dst++ = pose.q.x;
            *dst++ = pose.q.y;
            *dst++ = pose.q.z;
            *dst++ = pose.q.w;
        }
        else
        {
            const Subspace* subspace = mEntries[recIdx].subspace;
            const PxVec3 origin = subspace ? PxVec3(subspace->origin.x, subspace->origin.y, subspace->origin.z) :
                                             PxVec3(0.0f);
            *dst++ = pose.p.x - origin.x;
            *dst++ = pose.p.y - origin.y;
            *dst++ = pose.p.z - origin.z;
        }
    }
    return true;
}

bool CpuRigidBodyView::gatherVelAccColumnOvStage(const bool wantAngular,
                                                 const bool wantAcceleration,
                                                 const PxU32* outRecordIdx,
                                                 const PxU32 numOutputs,
                                                 const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    if (!dstTensor || !dstTensor->data)
        return false;

    const char* what = wantAcceleration ? (wantAngular ? "angular acceleration" : "linear acceleration") :
                                          (wantAngular ? "angular velocity" : "linear velocity");
    if (!checkTensorDevice(*dstTensor, -1, what, __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, what, __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, numOutputs * 3u, what, __FUNCTION__) ||
        !checkRecordIndices(outRecordIdx, numOutputs, mEntries.size(), what, __FUNCTION__))
    {
        return false;
    }

    float* dst = static_cast<float*>(dstTensor->data);
    for (PxU32 i = 0; i < numOutputs; i++)
    {
        const PxU32 recIdx = outRecordIdx ? outRecordIdx[i] : i;
        PxRigidBody* const body = mEntries[recIdx].body;
        const PxVec3 v = wantAcceleration ?
                             (wantAngular ? body->getAngularAcceleration() : body->getLinearAcceleration()) :
                             (wantAngular ? body->getAngularVelocity() : body->getLinearVelocity());
        *dst++ = v.x;
        *dst++ = v.y;
        *dst++ = v.z;
    }
    return true;
}

bool CpuRigidBodyView::getVelocities(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "velocity", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "velocity", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * 6u, "velocity", __FUNCTION__))
    {
        return false;
    }

    float* dst = static_cast<float*>(dstTensor->data);
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        PxVec3 linvel = mEntries[i].body->getLinearVelocity();
        PxVec3 angvel = mEntries[i].body->getAngularVelocity();
        *dst++ = linvel.x;
        *dst++ = linvel.y;
        *dst++ = linvel.z;
        *dst++ = angvel.x;
        *dst++ = angvel.y;
        *dst++ = angvel.z;
    }

    return true;
}

bool CpuRigidBodyView::getAccelerations(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "acceleration", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "acceleration", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * 6u, "acceleration", __FUNCTION__))
    {
        return false;
    }

    float* dst = static_cast<float*>(dstTensor->data);
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        PxVec3 linAcc = mEntries[i].body->getLinearAcceleration();
        PxVec3 angAcc = mEntries[i].body->getAngularAcceleration();
        *dst++ = linAcc.x;
        *dst++ = linAcc.y;
        *dst++ = linAcc.z;
        *dst++ = angAcc.x;
        *dst++ = angAcc.y;
        *dst++ = angAcc.z;
    }

    return true;
}

bool CpuRigidBodyView::setKinematicTargets(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);

    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "transform", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "transform", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * 7u, "transform", __FUNCTION__))
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, -1, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__) ||
            !checkIndexTensorSize(*indexTensor, getCount(), __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mAllIndices.data();
        numIndices = PxU32(mAllIndices.size());
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            RigidBodyEntry& entry = mEntries[idx];
            const float* src = static_cast<const float*>(srcTensor->data) + idx * 7;

            PxTransform target;
            target.p.x = *src++;
            target.p.y = *src++;
            target.p.z = *src++;
            target.q.x = *src++;
            target.q.y = *src++;
            target.q.z = *src++;
            target.q.w = *src++;

            Subspace* subspace = entry.subspace;
            if (subspace)
            {
                target.p.x += subspace->origin.x;
                target.p.y += subspace->origin.y;
                target.p.z += subspace->origin.z;
            }

            if (entry.type == RigidBodyType::eRigidDynamic && (entry.body->getRigidBodyFlags() & ::physx::PxRigidBodyFlag::eKINEMATIC))
            {
                static_cast<::physx::PxRigidDynamic*>(entry.body)->setKinematicTarget(target);
            }
            else
            {
                CARB_LOG_WARN("Cannot set kinematic target on articulation link or non-kinematic rigid body at '%s'", entry.path.c_str());
            }
        }
    }

    return true;
}

bool CpuRigidBodyView::setTransforms(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);

    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "transform", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "transform", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * 7u, "transform", __FUNCTION__))
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, -1, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__) ||
            !checkIndexTensorSize(*indexTensor, getCount(), __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mAllIndices.data();
        numIndices = PxU32(mAllIndices.size());
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            RigidBodyEntry& entry = mEntries[idx];
            const float* src = static_cast<const float*>(srcTensor->data) + idx * 7;

            PxTransform pose;
            pose.p.x = *src++;
            pose.p.y = *src++;
            pose.p.z = *src++;
            pose.q.x = *src++;
            pose.q.y = *src++;
            pose.q.z = *src++;
            pose.q.w = *src++;

            Subspace* subspace = entry.subspace;
            if (subspace)
            {
                pose.p.x += subspace->origin.x;
                pose.p.y += subspace->origin.y;
                pose.p.z += subspace->origin.z;
            }

            if (entry.type == RigidBodyType::eRigidDynamic)
            {
                entry.body->setGlobalPose(pose);
            }
            else if (mArticulations[idx])
            {
                // it's a root articulation link
                mArticulationCaches[idx]->rootLinkData->transform = pose;
                mArticulations[idx]->applyCache(*mArticulationCaches[idx], PxArticulationCacheFlag::eROOT_TRANSFORM);
            }
            else
            {
                CARB_LOG_WARN("Cannot assign transform to non-root articulation link at '%s'", entry.path.c_str());
            }
        }
    }

    return true;
}

bool CpuRigidBodyView::setVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);

    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "velocity", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "velocity", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * 6u, "velocity", __FUNCTION__))
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, -1, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__) ||
            !checkIndexTensorSize(*indexTensor, getCount(), __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mAllIndices.data();
        numIndices = PxU32(mAllIndices.size());
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            RigidBodyEntry& entry = mEntries[idx];
            const float* src = static_cast<const float*>(srcTensor->data) + idx * 6;

            PxVec3 linvel, angvel;
            linvel.x = *src++;
            linvel.y = *src++;
            linvel.z = *src++;
            angvel.x = *src++;
            angvel.y = *src++;
            angvel.z = *src++;

            if (entry.type == RigidBodyType::eRigidDynamic)
            {
                PxRigidDynamic* rd = static_cast<PxRigidDynamic*>(entry.body);
                rd->setLinearVelocity(linvel);
                rd->setAngularVelocity(angvel);
            }
            else if (mArticulations[idx])
            {
                // it's a root articulation link
                mArticulationCaches[idx]->rootLinkData->worldLinVel = linvel;
                mArticulationCaches[idx]->rootLinkData->worldAngVel = angvel;
                mArticulations[idx]->applyCache(*mArticulationCaches[idx], PxArticulationCacheFlag::eROOT_VELOCITIES);
            }
            else
            {
                CARB_LOG_WARN("Cannot assign velocities to rigid body at '%s'", entry.path.c_str());
            }
        }
    }

    return true;
}

void CpuRigidBodyView::prepareDirtyForceTracker()
{
    if (!mDirtyForceTracker)
    {
        mDirtyForceTracker = std::make_shared<CpuRigidBodyDirtyForceTracker>();

        PxU32 numBodies = getCount();
        mDirtyForceTracker->bodies.resize(numBodies);
        mDirtyForceTracker->dirtyFlags.resize(numBodies);

        for (PxU32 i = 0; i < numBodies; i++)
        {
            mDirtyForceTracker->bodies[i] = mEntries[i].body;
        }

        if (mCpuSimData)
        {
            mCpuSimData->addRigidBodyDirtyForceTracker(mDirtyForceTracker);
        }
    }
}

bool CpuRigidBodyView::applyForces(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);

    CARB_LOG_WARN("Deprecated function IArticulationView::applyForces, please use IArticulationView::applyForcesAndTorquesAtPosition instead.");
    return applyForcesAndTorquesAtPosition(srcTensor, nullptr, nullptr, indexTensor, true);
}

bool CpuRigidBodyView::applyForcesAndTorquesAtPosition(const TensorDesc* srcForceTensor,
                                                       const TensorDesc* srcTorqueTensor,
                                                       const TensorDesc* srcPositionTensor,
                                                       const TensorDesc* indexTensor,
                                                       const bool isGlobal)
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);

    bool validForceTensor = false;
    bool validTorqueTensor = false;
    bool validPositionTensor = false;
    bool hasForce = srcForceTensor && srcForceTensor->data;
    bool hasTorque = srcTorqueTensor && srcTorqueTensor->data;
    bool hasPosition = srcPositionTensor && srcPositionTensor->data;
    if (!hasForce && !hasTorque){
        CARB_LOG_WARN("No force or torque tensor is provided\n.");
        return false;
    }

    if (hasForce)
        validForceTensor = checkTensorDevice(*srcForceTensor, -1, "force", __FUNCTION__) &&
                           checkTensorFloat32(*srcForceTensor, "force", __FUNCTION__) &&
                           checkTensorSizeExact(*srcForceTensor, getCount() * 3u, "force", __FUNCTION__);

    if (hasTorque)
        validTorqueTensor = checkTensorDevice(*srcTorqueTensor, -1, "torque", __FUNCTION__) &&
                            checkTensorFloat32(*srcTorqueTensor, "torque", __FUNCTION__) &&
                            checkTensorSizeExact(*srcTorqueTensor, getCount() * 3u, "torque", __FUNCTION__);

    if (!validForceTensor && !validTorqueTensor)
    {
        CARB_LOG_WARN("No correct force or torque tensor is provided\n.");
        return false;
    }

    if (hasPosition)
    {
        if (!validForceTensor)
        {
            CARB_LOG_ERROR("Received a position tensor wihtout a compatible force tensor.");
            return false;
        }
        validPositionTensor = checkTensorDevice(*srcPositionTensor, -1, "position", __FUNCTION__) &&
                              checkTensorFloat32(*srcPositionTensor, "position", __FUNCTION__) &&
                              checkTensorSizeExact(*srcPositionTensor, getCount() * 3u, "position", __FUNCTION__);
        if (!validPositionTensor)
            return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, -1, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__) ||
            !checkIndexTensorSize(*indexTensor, getCount(), __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mAllIndices.data();
        numIndices = PxU32(mAllIndices.size());
    }

    prepareDirtyForceTracker();

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            RigidBodyEntry& entry = mEntries[idx];
            // do not apply forces to kinematic actors - it crashes physx
            if (!entry.body->getRigidBodyFlags().isSet(PxRigidBodyFlag::eKINEMATIC))
            {
                if (validForceTensor)
                {
                    const float* src = static_cast<const float*>(srcForceTensor->data) + idx * 3;
                    PxVec3 force(src[0], src[1], src[2]);
                    if (!isGlobal)
                    {
                        // translate force vector into global space
                        PxTransform pose = entry.body->getGlobalPose();
                        force = pose.q.rotate(force);
                    }
                    entry.body->addForce(force);
                    mDirtyForceTracker->dirtyFlags[idx] |= RigidBodyDirtyForceFlags::eForce;
                    if (validPositionTensor)
                    {
                        PxTransform pose = entry.body->getGlobalPose();
                        const PxVec3 com = pose.transform(entry.body->getCMassLocalPose().p);
                        const float* srcP = static_cast<const float*>(srcPositionTensor->data) + idx * 3;
                        PxVec3 position(srcP[0], srcP[1], srcP[2]);
                        if (!isGlobal)
                            position = pose.transform(position);
                        PxVec3 tmp = (position - com).cross(force);
                        entry.body->addTorque((position - com).cross(force));
                        mDirtyForceTracker->dirtyFlags[idx] |= RigidBodyDirtyForceFlags::eTorque;
                    }
                }
                if (validTorqueTensor)
                {
                    const float* src = static_cast<const float*>(srcTorqueTensor->data) + idx * 3;
                    PxVec3 torque;
                    torque.x = src[0];
                    torque.y = src[1];
                    torque.z = src[2];
                    if (!isGlobal)
                    {
                        // translate force vector into global space
                        PxTransform pose = entry.body->getGlobalPose();
                        torque = pose.q.rotate(torque);
                    }
                    entry.body->addTorque(torque);
                    mDirtyForceTracker->dirtyFlags[idx] |= RigidBodyDirtyForceFlags::eTorque;
                }
            }
        }
    }

    mDirtyForceTracker->isDirty = true;

    return true;
}

// ── Mask helpers & masked variants ──────────────────────────────────────────

using omni::physics::tensors::MaskResult;
using omni::physics::tensors::resolveMaskToIndices;
using omni::physics::tensors::makeIndexTensorDesc;

bool CpuRigidBodyView::setKinematicTargetsMasked(const TensorDesc* src, const TensorDesc* mask)
{
    std::vector<uint32_t> indices;
    auto result = resolveMaskToIndices(mask, getCount(), -1, indices, __FUNCTION__);
    if (result == MaskResult::Error) return false;
    if (result == MaskResult::Empty) return true;
    if (result == MaskResult::All)   return setKinematicTargets(src, nullptr);
    TensorDesc idx = makeIndexTensorDesc(indices, -1);
    return setKinematicTargets(src, &idx);
}

bool CpuRigidBodyView::setTransformsMasked(const TensorDesc* src, const TensorDesc* mask)
{
    std::vector<uint32_t> indices;
    auto result = resolveMaskToIndices(mask, getCount(), -1, indices, __FUNCTION__);
    if (result == MaskResult::Error) return false;
    if (result == MaskResult::Empty) return true;
    if (result == MaskResult::All)   return setTransforms(src, nullptr);
    TensorDesc idx = makeIndexTensorDesc(indices, -1);
    return setTransforms(src, &idx);
}

bool CpuRigidBodyView::setVelocitiesMasked(const TensorDesc* src, const TensorDesc* mask)
{
    std::vector<uint32_t> indices;
    auto result = resolveMaskToIndices(mask, getCount(), -1, indices, __FUNCTION__);
    if (result == MaskResult::Error) return false;
    if (result == MaskResult::Empty) return true;
    if (result == MaskResult::All)   return setVelocities(src, nullptr);
    TensorDesc idx = makeIndexTensorDesc(indices, -1);
    return setVelocities(src, &idx);
}

bool CpuRigidBodyView::applyForcesMasked(const TensorDesc* src, const TensorDesc* mask)
{
    std::vector<uint32_t> indices;
    auto result = resolveMaskToIndices(mask, getCount(), -1, indices, __FUNCTION__);
    if (result == MaskResult::Error) return false;
    if (result == MaskResult::Empty) return true;
    if (result == MaskResult::All)   return applyForces(src, nullptr);
    TensorDesc idx = makeIndexTensorDesc(indices, -1);
    return applyForces(src, &idx);
}

bool CpuRigidBodyView::applyForcesAndTorquesAtPositionMasked(const TensorDesc* srcForceTensor,
                                                              const TensorDesc* srcTorqueTensor,
                                                              const TensorDesc* srcPositionTensor,
                                                              const TensorDesc* maskTensor,
                                                              const bool isGlobal)
{
    std::vector<uint32_t> indices;
    auto result = resolveMaskToIndices(maskTensor, getCount(), -1, indices, __FUNCTION__);
    if (result == MaskResult::Error) return false;
    if (result == MaskResult::Empty) return true;
    if (result == MaskResult::All)   return applyForcesAndTorquesAtPosition(srcForceTensor, srcTorqueTensor, srcPositionTensor, nullptr, isGlobal);
    TensorDesc idx = makeIndexTensorDesc(indices, -1);
    return applyForcesAndTorquesAtPosition(srcForceTensor, srcTorqueTensor, srcPositionTensor, &idx, isGlobal);
}

bool CpuRigidBodyView::setMassesMasked(const TensorDesc* src, const TensorDesc* mask)
{
    std::vector<uint32_t> indices;
    auto result = resolveMaskToIndices(mask, getCount(), -1, indices, __FUNCTION__);
    if (result == MaskResult::Error) return false;
    if (result == MaskResult::Empty) return true;
    if (result == MaskResult::All)   return setMasses(src, nullptr);
    TensorDesc idx = makeIndexTensorDesc(indices, -1);
    return setMasses(src, &idx);
}

bool CpuRigidBodyView::setCOMsMasked(const TensorDesc* src, const TensorDesc* mask)
{
    std::vector<uint32_t> indices;
    auto result = resolveMaskToIndices(mask, getCount(), -1, indices, __FUNCTION__);
    if (result == MaskResult::Error) return false;
    if (result == MaskResult::Empty) return true;
    if (result == MaskResult::All)   return setCOMs(src, nullptr);
    TensorDesc idx = makeIndexTensorDesc(indices, -1);
    return setCOMs(src, &idx);
}

bool CpuRigidBodyView::setInertiasMasked(const TensorDesc* src, const TensorDesc* mask)
{
    std::vector<uint32_t> indices;
    auto result = resolveMaskToIndices(mask, getCount(), -1, indices, __FUNCTION__);
    if (result == MaskResult::Error) return false;
    if (result == MaskResult::Empty) return true;
    if (result == MaskResult::All)   return setInertias(src, nullptr);
    TensorDesc idx = makeIndexTensorDesc(indices, -1);
    return setInertias(src, &idx);
}

// setDisable*/material/rest/contact/compliant Masked: BaseRigidBodyView


}
}
}
