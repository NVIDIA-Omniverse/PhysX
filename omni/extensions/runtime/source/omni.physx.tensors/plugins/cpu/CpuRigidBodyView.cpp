// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include <UsdPCH.h>
// clang-format on

#include "CpuRigidBodyView.h"
#include "CpuSimulationView.h"

#include "../GlobalsAreBad.h"
#include "../CommonTypes.h"

#include <carb/logging/Log.h>

#include <PxPhysicsAPI.h>
#include <omni/physics/tensors/TensorUtils.h>

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
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__))
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
                CARB_LOG_WARN("Cannot set kinematic target on articulation link or non-kinematic rigid body at '%s'", entry.path.GetText());
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
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__))
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
                CARB_LOG_WARN("Cannot assign transform to non-root articulation link at '%s'", entry.path.GetText());
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
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__))
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
                CARB_LOG_WARN("Cannot assign velocities to rigid body at '%s'", entry.path.GetText());
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
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__))
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
                    // printf("CPU idx %u force = %f,%f,%f\n", idx, force.x, force.y, force.z);
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
                        // printf("CPU idx %u torque = %f,%f,%f\n", idx, tmp.x, tmp.y, tmp.z);
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
}
}
}
