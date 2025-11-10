// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include <UsdPCH.h>
// clang-format on

#include "CpuArticulationView.h"
#include "CpuSimulationView.h"

#include "../GlobalsAreBad.h"
#include "../CommonTypes.h"

#include <PxPhysicsAPI.h>

#include <carb/logging/Log.h>
#include <omni/physx/IPhysx.h>

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

CpuArticulationView::CpuArticulationView(CpuSimulationView* sim, const std::vector<ArticulationEntry>& entries)
    : BaseArticulationView(sim, entries)
{
    PxU32 numArtis = PxU32(mEntries.size());

    // initialize articulation cache
    mArticulationCaches.resize(numArtis);
    for (PxU32 i = 0; i < numArtis; i++)
    {
        PxArticulationReducedCoordinate* arti = mEntries[i].arti;
        mArticulationCaches[i] = arti->createCache();
        if (mArticulationCaches[i])
        {
            arti->copyInternalStateToCache(*mArticulationCaches[i], PxArticulationCacheFlag::eALL);
        }
    }
    mCpuSimData = sim->getCpuSimulationData();
}

CpuArticulationView::~CpuArticulationView()
{
}

bool CpuArticulationView::getLinkTransforms(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "link transform", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "link transform", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxLinks * 7u, "link transform", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        auto& links = mEntries[i].links;
        Subspace* subspace = mEntries[i].subspace;
        float* dst = static_cast<float*>(dstTensor->data) + i * mMaxLinks * 7;
        for (PxU32 j = 0; j < mEntries[i].numLinks; j++)
        {
            PxTransform pose = links[j]->getGlobalPose();
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
    }

    return true;
}

bool CpuArticulationView::getLinkVelocities(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "link velocity", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "link velocity", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxLinks * 6u, "link velocity", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        auto& links = mEntries[i].links;
        float* dst = static_cast<float*>(dstTensor->data) + i * mMaxLinks * 6;
        for (PxU32 j = 0; j < mEntries[i].numLinks; j++)
        {
            PxVec3 linvel = links[j]->getLinearVelocity();
            PxVec3 angvel = links[j]->getAngularVelocity();
            *dst++ = linvel.x;
            *dst++ = linvel.y;
            *dst++ = linvel.z;
            *dst++ = angvel.x;
            *dst++ = angvel.y;
            *dst++ = angvel.z;
        }
    }

    return true;
}

bool CpuArticulationView::getLinkAccelerations(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "link acceleration", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "link acceleration", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxLinks * 6u, "link acceleration", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {

        PxArticulationReducedCoordinate* arti = mEntries[i].arti;
        PxArticulationCache* cache = mArticulationCaches[i];
        if (cache && arti)
        {
            arti->copyInternalStateToCache(*cache, PxArticulationCacheFlag::eLINK_ACCELERATION);
            float* dst = static_cast<float*>(dstTensor->data) + i * mMaxLinks * 6;
            for (PxU32 j = 0; j < mEntries[i].numLinks; j++)
            {
                PxVec3 linAcc = cache->linkAcceleration[j].linear;
                PxVec3 angAcc = cache->linkAcceleration[j].angular;
                *dst++ = linAcc.x;
                *dst++ = linAcc.y;
                *dst++ = linAcc.z;
                *dst++ = angAcc.x;
                *dst++ = angAcc.y;
                *dst++ = angAcc.z;
            }
        }
    }

    return true;
}

bool CpuArticulationView::getRootTransforms(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "root transform", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "root transform", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * 7u, "root transform", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        PxArticulationReducedCoordinate* arti = mEntries[i].arti;
        PxArticulationCache* cache = mArticulationCaches[i];
        if (arti && cache)
        {
            arti->copyInternalStateToCache(*cache, PxArticulationCacheFlag::eROOT_TRANSFORM);
            PxTransform pose = cache->rootLinkData->transform;
            Subspace* subspace = mEntries[i].subspace;
            if (subspace)
            {
                pose.p.x -= subspace->origin.x;
                pose.p.y -= subspace->origin.y;
                pose.p.z -= subspace->origin.z;
            }
            float* dst = static_cast<float*>(dstTensor->data) + i * 7;
            *dst++ = pose.p.x;
            *dst++ = pose.p.y;
            *dst++ = pose.p.z;
            *dst++ = pose.q.x;
            *dst++ = pose.q.y;
            *dst++ = pose.q.z;
            *dst++ = pose.q.w;
        }
    }

    return true;
}

bool CpuArticulationView::getRootVelocities(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "root velocity", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "root velocity", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * 6u, "root velocity", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        PxArticulationReducedCoordinate* arti = mEntries[i].arti;
        PxArticulationCache* cache = mArticulationCaches[i];
        if (arti && cache)
        {
            arti->copyInternalStateToCache(*cache, PxArticulationCacheFlag::eROOT_VELOCITIES);
            const PxVec3& linvel = cache->rootLinkData->worldLinVel;
            const PxVec3& angvel = cache->rootLinkData->worldAngVel;
            float* dst = static_cast<float*>(dstTensor->data) + i * 6;
            *dst++ = linvel.x;
            *dst++ = linvel.y;
            *dst++ = linvel.z;
            *dst++ = angvel.x;
            *dst++ = angvel.y;
            *dst++ = angvel.z;
        }
    }

    return true;
}

bool CpuArticulationView::setRootTransforms(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    
    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "root transform", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "root transform", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * 7u, "root transform", __FUNCTION__))
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
            PxArticulationReducedCoordinate* arti = mEntries[idx].arti;
            PxArticulationCache* cache = mArticulationCaches[idx];
            if (arti && cache)
            {
                const float* src = static_cast<const float*>(srcTensor->data) + idx * 7;
                PxTransform& pose = cache->rootLinkData->transform;
                pose.p.x = *src++;
                pose.p.y = *src++;
                pose.p.z = *src++;
                pose.q.x = *src++;
                pose.q.y = *src++;
                pose.q.z = *src++;
                pose.q.w = *src++;

                Subspace* subspace = mEntries[idx].subspace;
                if (subspace)
                {
                    pose.p.x += subspace->origin.x;
                    pose.p.y += subspace->origin.y;
                    pose.p.z += subspace->origin.z;
                }
                arti->applyCache(*cache, PxArticulationCacheFlag::eROOT_TRANSFORM);
            }
        }
    }

    return true;
}

bool CpuArticulationView::setRootVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);

    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "root velocity", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "root velocity", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * 6u, "root velocity", __FUNCTION__))
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
            PxArticulationReducedCoordinate* arti = mEntries[idx].arti;
            PxArticulationCache* cache = mArticulationCaches[idx];
            if (arti && cache)
            {
                const float* src = static_cast<const float*>(srcTensor->data) + idx * 6;
                PxVec3& linvel = cache->rootLinkData->worldLinVel;
                PxVec3& angvel = cache->rootLinkData->worldAngVel;
                linvel.x = *src++;
                linvel.y = *src++;
                linvel.z = *src++;
                angvel.x = *src++;
                angvel.y = *src++;
                angvel.z = *src++;

                arti->applyCache(*cache, PxArticulationCacheFlag::eROOT_VELOCITIES);
            }
        }
    }

    return true;
}

bool CpuArticulationView::getDofPositions(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "DOF position", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "DOF position", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxDofs, "DOF position", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        PxArticulationReducedCoordinate* arti = mEntries[i].arti;
        PxArticulationCache* cache = mArticulationCaches[i];
        if (arti && cache)
        {
            arti->copyInternalStateToCache(*cache, PxArticulationCacheFlag::ePOSITION);
            float* dst = static_cast<float*>(dstTensor->data) + i * mMaxDofs;
            for (PxU32 j = 0; j < mEntries[i].numDofs; j++)
            {
                dst[j] = mEntries[i].metatype->isDofBody0Parent(j) ? cache->jointPosition[j] : -cache->jointPosition[j];
            }
        }
    }

    return true;
}

bool CpuArticulationView::getDofVelocities(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "DOF velocity", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "DOF velocity", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxDofs, "DOF velocity", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        PxArticulationReducedCoordinate* arti = mEntries[i].arti;
        PxArticulationCache* cache = mArticulationCaches[i];
        if (arti && cache)
        {
            arti->copyInternalStateToCache(*cache, PxArticulationCacheFlag::eVELOCITY);
            float* dst = static_cast<float*>(dstTensor->data) + i * mMaxDofs;
            for (PxU32 j = 0; j < mEntries[i].numDofs; j++)
            {
                dst[j] = mEntries[i].metatype->isDofBody0Parent(j) ? cache->jointVelocity[j] : -cache->jointVelocity[j];
            }
        }
    }

    return true;
}

bool CpuArticulationView::setDofPositions(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(srcTensor);

    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "DOF position", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "DOF position", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * mMaxDofs, "DOF position", __FUNCTION__))
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
            PxArticulationReducedCoordinate* arti = mEntries[idx].arti;
            PxArticulationCache* cache = mArticulationCaches[idx];
            if (arti && cache)
            {
                const float* src = static_cast<const float*>(srcTensor->data) + idx * mMaxDofs;
                for (PxU32 j = 0; j < mEntries[idx].numDofs; j++)
                {
                    cache->jointPosition[j] = mEntries[idx].metatype->isDofBody0Parent(j) ? src[j] : -src[j];
                }

                arti->applyCache(*cache, PxArticulationCacheFlag::ePOSITION);
            }
        }
    }

    return true;
}

bool CpuArticulationView::setDofVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(srcTensor);

    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "DOF velocity", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "DOF velocity", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * mMaxDofs, "DOF velocity", __FUNCTION__))
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
            PxArticulationReducedCoordinate* arti = mEntries[idx].arti;
            PxArticulationCache* cache = mArticulationCaches[idx];
            if (arti && cache)
            {
                const float* src = static_cast<const float*>(srcTensor->data) + idx * mMaxDofs;
                for (PxU32 j = 0; j < mEntries[idx].numDofs; j++)
                {
                    cache->jointVelocity[j] = mEntries[idx].metatype->isDofBody0Parent(j) ? src[j] : -src[j];
                }
                arti->applyCache(*cache, PxArticulationCacheFlag::eVELOCITY);
            }
        }
    }

    return true;
}

bool CpuArticulationView::setDofActuationForces(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(srcTensor);

    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "DOF force", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "DOF force", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * mMaxDofs, "DOF force", __FUNCTION__))
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
            PxArticulationReducedCoordinate* arti = mEntries[idx].arti;
            PxArticulationCache* cache = mArticulationCaches[idx];
            if (arti && cache)
            {
                const float* src = static_cast<const float*>(srcTensor->data) + idx * mMaxDofs;
                for (PxU32 j = 0; j < mEntries[idx].numDofs; j++)
                {
                    cache->jointForce[j] = mEntries[idx].metatype->isDofBody0Parent(j) ? src[j] : -src[j];
                }
                arti->applyCache(*cache, PxArticulationCacheFlag::eFORCE);
            }
        }
    }

    return true;
}

bool CpuArticulationView::setDofPositionTargets(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(srcTensor);

    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "DOF target", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "DOF target", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * mMaxDofs, "DOF target", __FUNCTION__))
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
            ArticulationEntry& entry = mEntries[idx];
            const float* src = static_cast<const float*>(srcTensor->data) + idx * mMaxDofs;
            for (PxU32 j = 0; j < entry.numDofs; j++)
            {
                const DofImpl& dofImpl = entry.dofImpls[j];
                if (dofImpl.joint)
                {
                    dofImpl.joint->setDriveTarget(
                        dofImpl.axis, mEntries[idx].metatype->isDofBody0Parent(j) ? src[j] : -src[j]);
                }
            }
        }
    }

    return true;
}

bool CpuArticulationView::setDofVelocityTargets(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(srcTensor);

    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "DOF target", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "DOF target", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * mMaxDofs, "DOF target", __FUNCTION__))
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
            ArticulationEntry& entry = mEntries[idx];
            const float* src = static_cast<const float*>(srcTensor->data) + idx * mMaxDofs;
            for (PxU32 j = 0; j < entry.numDofs; j++)
            {
                const DofImpl& dofImpl = entry.dofImpls[j];
                if (dofImpl.joint)
                {
                    dofImpl.joint->setDriveVelocity(dofImpl.axis, mEntries[idx].metatype->isDofBody0Parent(j) ? src[j] : -src[j]);
                }
            }
        }
    }

    return true;
}

bool CpuArticulationView::getDofPositionTargets(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "DOF target", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "DOF target", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxDofs, "DOF target", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        float* dst = static_cast<float*>(dstTensor->data) + i * mMaxDofs;
        for (PxU32 j = 0; j < mEntries[i].numDofs; j++)
        {
            const DofImpl& dofImpl = mEntries[i].dofImpls[j];
            if (dofImpl.joint)
            {
                dst[j] = mEntries[i].metatype->isDofBody0Parent(j) ? dofImpl.joint->getDriveTarget(dofImpl.axis) :
                                                                      -dofImpl.joint->getDriveTarget(dofImpl.axis);
            }
            else
            {
                dst[j] = 0.0f;
            }
        }
    }

    return true;
}

bool CpuArticulationView::getDofVelocityTargets(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "DOF target", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "DOF target", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxDofs, "DOF target", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        float* dst = static_cast<float*>(dstTensor->data) + i * mMaxDofs;
        for (PxU32 j = 0; j < mEntries[i].numDofs; j++)
        {
            const DofImpl& dofImpl = mEntries[i].dofImpls[j];
            if (dofImpl.joint)
            {
                dst[j] = mEntries[i].metatype->isDofBody0Parent(j) ? dofImpl.joint->getDriveVelocity(dofImpl.axis) :
                                                                      -dofImpl.joint->getDriveVelocity(dofImpl.axis);
            }
            else
            {
                dst[j] = 0.0f;
            }
        }
    }

    return true;
}


bool CpuArticulationView::getDofActuationForces(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "dof actuation force", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "dof actuation force", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxDofs, "dof actuation force", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        PxArticulationReducedCoordinate* arti = mEntries[i].arti;
        PxArticulationCache* cache = mArticulationCaches[i];
        if (arti && cache)
        {
            arti->copyInternalStateToCache(*cache, PxArticulationCacheFlag::eFORCE);
            float* dst = static_cast<float*>(dstTensor->data) + i * mMaxDofs;
            for (PxU32 j = 0; j < mEntries[i].numDofs; j++)
            {
                dst[j] = mEntries[i].metatype->isDofBody0Parent(j) ? cache->jointForce[j] : -cache->jointForce[j];
            }
        }
    }

    return true;
}


bool CpuArticulationView::getJacobians(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    uint32_t jacobianRows = 0;
    uint32_t jacobianCols = 0;

    // this will fail if view is not homogeneous
    if (!getJacobianShape(&jacobianRows, &jacobianCols))
    {
        return false;
    }

    uint32_t jacobianSize = jacobianRows * jacobianCols;

    if (!checkTensorDevice(*dstTensor, -1, "Jacobian", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "Jacobian", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * jacobianSize, "Jacobian", __FUNCTION__))
    {
        return false;
    }

    float* dst = (float*)dstTensor->data;

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        PxArticulationReducedCoordinate* arti = mEntries[i].arti;
        PxArticulationCache* cache = mArticulationCaches[i];

        PxU32 nRows, nCols;
        arti->commonInit();
        arti->computeDenseJacobian(*cache, nRows, nCols);

        // extra safety check
        // this could fail if articulation structure or fixed base changed
        if (nRows == jacobianRows && nCols == jacobianCols)
        {
            for (PxU32 j = 0; j < jacobianSize; j++)
            {
                dst[j] = cache->denseJacobian[j];
            }
        }
        else
        {
            CARB_LOG_ERROR("Invalid Jacobian shape for destination tensor!");
            memset(dst, 0, jacobianSize * sizeof(float));
        }

        dst += jacobianSize;
    }

    return true;
}

// DEPRECATED
bool CpuArticulationView::getMassMatrices(const TensorDesc* dstTensor) const
{
    CARB_LOG_WARN("DEPRECATED: Please use getGeneralizedMassMatrices instead.");
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    uint32_t massMatrixRows = 0;
    uint32_t massMatrixCols = 0;

    // this will fail if view is not homogeneous
    if (!getMassMatrixShape(&massMatrixRows, &massMatrixCols))
    {
        return false;
    }

    uint32_t massMatrixSize = massMatrixRows * massMatrixCols;

    if (!checkTensorDevice(*dstTensor, -1, "Mass Matrix", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "Mass Matrix", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * massMatrixSize, "Mass Matrix", __FUNCTION__))
    {
        return false;
    }

    float* dst = (float*)dstTensor->data;

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        PxArticulationReducedCoordinate* arti = mEntries[i].arti;
        PxArticulationCache* cache = mArticulationCaches[i];

        arti->commonInit();
         arti->computeGeneralizedMassMatrix(*cache);

        for (PxU32 j = 0; j < massMatrixSize; j++)
        {
            dst[j] = cache->massMatrix[j];
        }

        dst += massMatrixSize;
    }

    return true;
}

bool CpuArticulationView::getGeneralizedMassMatrices(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    const bool isFixedBase = mEntries[0].metatype->getFixedBase();
    const PxU32 maxDofs = isFixedBase ? mMaxDofs : mMaxDofs + 6;

    uint32_t massMatrixRows = 0;
    uint32_t massMatrixCols = 0;

    // this will fail if view is not homogeneous
    if (!getMassMatrixShape(&massMatrixRows, &massMatrixCols))
    {
        return false;
    }

    if (!isFixedBase)
    {
        massMatrixRows += 6;
        massMatrixCols += 6;
    }
    uint32_t massMatrixSize = massMatrixRows * massMatrixCols;

    if (!checkTensorDevice(*dstTensor, -1, "Mass Matrix", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "Mass Matrix", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * massMatrixSize, "Mass Matrix", __FUNCTION__))
    {
        return false;
    }

    float* dst = (float*)dstTensor->data;

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        PxArticulationReducedCoordinate* arti = mEntries[i].arti;
        PxArticulationCache* cache = mArticulationCaches[i];
        arti->commonInit();
        arti->computeMassMatrix(*cache);

        for (PxU32 j = 0; j < massMatrixSize; j++)
        {
            dst[j] = cache->massMatrix[j];
        }

        dst += massMatrixSize;
    }

    return true;
}

// DEPRECATED
bool CpuArticulationView::getCoriolisAndCentrifugalForces(const TensorDesc* dstTensor) const
{
    CARB_LOG_WARN("DEPRECATED: Please use getCoriolisAndCentrifugalCompensationForces instead.");
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "coriolis and centrifugal forces", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "coriolis and centrifugal forces", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxDofs, "coriolis and centrifugal forces", __FUNCTION__))
    {
        return false;
    }

    float* dst = (float*)dstTensor->data;

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        PxArticulationReducedCoordinate* arti = mEntries[i].arti;
        PxArticulationCache* cache = mArticulationCaches[i];

        arti->commonInit();
        arti->computeCoriolisAndCentrifugalForce(*cache);

        for (PxU32 j = 0; j < mMaxDofs; j++)
        {
            dst[j] = mEntries[i].metatype->isDofBody0Parent(j) ? cache->jointForce[j] : -cache->jointForce[j];
        }

        dst += mMaxDofs;
    }

    return true;
}

bool CpuArticulationView::getCoriolisAndCentrifugalCompensationForces(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    const bool isFixedBase = mEntries[0].metatype->getFixedBase();
    const PxU32 maxDofs = isFixedBase ? mMaxDofs : mMaxDofs + 6;

    if (!checkTensorDevice(*dstTensor, -1, "coriolis and centrifugal forces", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "coriolis and centrifugal forces", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * maxDofs, "coriolis and centrifugal forces", __FUNCTION__))
    {
        return false;
    }

    float* dst = (float*)dstTensor->data;

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        PxArticulationReducedCoordinate* arti = mEntries[i].arti;
        PxArticulationCache* cache = mArticulationCaches[i];

        arti->commonInit();
        arti->computeCoriolisCompensation(*cache);
        if (isFixedBase)
        {
            for (PxU32 j = 0; j < mMaxDofs; j++)
            {
                dst[j] = mEntries[i].metatype->isDofBody0Parent(j) ? cache->coriolisForce[j] : -cache->coriolisForce[j];
            }
        }
        else
        {
            for (PxU32 j = 0; j < 6; j++)
            {
                dst[j] = cache->coriolisForce[j];
            }
            for (PxU32 j = 6; j < maxDofs; j++)
            {
                dst[j] = mEntries[i].metatype->isDofBody0Parent(j - 6) ? cache->coriolisForce[j] : -cache->coriolisForce[j];
            }
        }

        dst += maxDofs;
    }

    return true;
}

// DEPRECATED
bool CpuArticulationView::getGeneralizedGravityForces(const TensorDesc* dstTensor) const
{
    CARB_LOG_WARN("DEPRECATED: Please use getGravityCompensationForces instead.");
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "generalized gravity forces", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "generalized gravity forces", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxDofs, "generalized gravity forces", __FUNCTION__))
    {
        return false;
    }

    float* dst = (float*)dstTensor->data;

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        PxArticulationReducedCoordinate* arti = mEntries[i].arti;
        PxArticulationCache* cache = mArticulationCaches[i];

        arti->commonInit();
        arti->computeGeneralizedGravityForce(*cache);

        for (PxU32 j = 0; j < mMaxDofs; j++)
        {
            dst[j] = mEntries[i].metatype->isDofBody0Parent(j) ? cache->jointForce[j] : -cache->jointForce[j];
        }

        dst += mMaxDofs;
    }

    return true;
}

bool CpuArticulationView::getGravityCompensationForces(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    const bool isFixedBase = mEntries[0].metatype->getFixedBase();
    const PxU32 maxDofs = isFixedBase ? mMaxDofs : mMaxDofs + 6;

    if (!checkTensorDevice(*dstTensor, -1, "gravity compensation forces", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "gravity compensation forces", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * maxDofs, "gravity compensation forces", __FUNCTION__))
    {
        return false;
    }

    float* dst = (float*)dstTensor->data;

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        PxArticulationReducedCoordinate* arti = mEntries[i].arti;
        PxArticulationCache* cache = mArticulationCaches[i];

        arti->commonInit();
        arti->computeGravityCompensation(*cache);
        if (isFixedBase)
        {
            for (PxU32 j = 0; j < mMaxDofs; j++)
            {
                dst[j] = mEntries[i].metatype->isDofBody0Parent(j) ? cache->gravityCompensationForce[j] : -cache->gravityCompensationForce[j];
            }
        }
        else
        {
            for (PxU32 j = 0; j < 6; j++)
            {
                dst[j] = cache->gravityCompensationForce[j];
            }
            for (PxU32 j = 6; j < maxDofs; j++)
            {
                dst[j] = mEntries[i].metatype->isDofBody0Parent(j - 6) ? cache->gravityCompensationForce[j] : -cache->gravityCompensationForce[j];
            }
        }

        dst += maxDofs;
    }

    return true;
}

bool CpuArticulationView::getArticulationMassCenter(const TensorDesc* dstTensor, bool localFrame) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "Articulation Mass Center", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "Articulation Mass Center", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * 3, "Articulation Mass Center", __FUNCTION__))
    {
        return false;
    }

    PxVec3* dst = static_cast<PxVec3*>(dstTensor->data);
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        dst[i] = mEntries[i].arti->computeArticulationCOM(localFrame);
    }

    return true;
}

bool CpuArticulationView::getArticulationCentroidalMomentum(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);
    const bool isFixedBase = mEntries[0].metatype->getFixedBase();
    if (isFixedBase)
    {
        CARB_LOG_ERROR("Articulation has fixed base, centroidal momentum is not defined");
        return false;
    }

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "Articulation Centroidal Momentum", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "Articulation Centroidal Momentum", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * 6 * (mMaxDofs + 7), "Articulation Centroidal Momentum", __FUNCTION__))
    {
        return false;
    }


    PxReal* dst = (PxReal*)dstTensor->data;
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        PxArticulationReducedCoordinate* arti = mEntries[i].arti;
        PxArticulationCache* cache = mArticulationCaches[i];
        arti->commonInit();
        arti->computeMassMatrix(*cache);
        arti->computeCoriolisCompensation(*cache);
        arti->computeCentroidalMomentumMatrix(*cache);
        for (PxU32 row = 0; row < 6; ++row)
        {
            PxU32 startIdx = i * 6 * (mMaxDofs + 7);
            // root + dofs
            for (PxU32 j = 0; j < 6 + mMaxDofs; ++j)
            {
                dst[startIdx + row * (mMaxDofs + 7) + j] = cache->centroidalMomentumMatrix[row * (mMaxDofs + 6) + j];
            }
            //bias force
            dst[startIdx + row * (mMaxDofs + 7) + 6 + mMaxDofs] = cache->centroidalMomentumBias[row];
        }
    }

    return true;
}

static void transformToParentFrame(const PxArticulationJointReducedCoordinate* joint, PxVec3& force, PxVec3& torque)
{
    // joint child/parent frame representation in the global frame
    PxTransform GpLp = joint->getParentArticulationLink().getGlobalPose() * joint->getParentPose();
    PxTransform GcLc = joint->getChildArticulationLink().getGlobalPose() * joint->getChildPose();
    // child to parent rotation and translation
    PxQuat J = GpLp.q.getConjugate() * GcLc.q;
    PxVec3 d = GpLp.p - GcLc.p; // global frame
    d = GcLc.q.rotateInv(d); // local frame
    // torque needs further modification before transforming to the parent frame
    torque = -1.0f * J.rotate(torque - d.cross(force));
    force = -1.0f * J.rotate(force);
    // printf("parent force= %f,%f,%f torque= %f,%f,%f\n", force.x, force.y, force.z, torque.x, torque.y, torque.z);
}

bool CpuArticulationView::getLinkIncomingJointForce(const TensorDesc* dstTensor) const
{
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "link incoming joint force", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "link incoming joint force", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxLinks * 6u, "link incoming joint force", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        PxArticulationReducedCoordinate* arti = mEntries[i].arti;
        PxArticulationCache* cache = mArticulationCaches[i];
        if (arti && cache)
        {
            arti->copyInternalStateToCache(*cache, PxArticulationCacheFlag::eLINK_INCOMING_JOINT_FORCE);
            float* dst = static_cast<float*>(dstTensor->data) + i * mMaxLinks * 6;
            for (PxU32 j = 0; j < mEntries[i].numLinks; ++j)
            {
                PxArticulationLink* link = mEntries[i].links[j];
                bool isJointBody0Parent = mEntries[i].isIncomingJointBody0Parent[j];
                PxQuat physxToUsdRotation = mEntries[i].incomingJointPhysxToUsdRotations[j];
                CARB_ASSERT(physxToUsdRotation.isUnit());

                PxSpatialForce sf = cache->linkIncomingJointForce[j];
                if (!isJointBody0Parent)
                {
                    PxArticulationJointReducedCoordinate* joint = link->getInboundJoint();
                    if (joint)
                    {
                        transformToParentFrame(joint, sf.force, sf.torque);
                    }
                }
                PxVec3 usdSpaceForce = physxToUsdRotation.rotate(sf.force);
                PxVec3 usdSpaceTorque = physxToUsdRotation.rotate(sf.torque);
                
                *dst++ = usdSpaceForce.x;
                *dst++ = usdSpaceForce.y;
                *dst++ = usdSpaceForce.z;
                *dst++ = usdSpaceTorque.x;
                *dst++ = usdSpaceTorque.y;
                *dst++ = usdSpaceTorque.z;
            }
        }
    }

    return true;
}

bool CpuArticulationView::getDofProjectedJointForces(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "dof projected joint force", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "dof projected joint force", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxDofs, "dof projected joint force", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        PxArticulationReducedCoordinate* arti = mEntries[i].arti;
        PxArticulationCache* cache = mArticulationCaches[i];

        if (arti && cache)
        {
            arti->copyInternalStateToCache(*cache, PxArticulationCacheFlag::eLINK_INCOMING_JOINT_FORCE);
            float* dst = static_cast<float*>(dstTensor->data) + i * mMaxDofs;
            for (PxU32 j = 0; j < mEntries[i].numLinks; ++j)
            {
                PxArticulationLink* link = mEntries[i].links[j];
                PxArticulationJointReducedCoordinate* joint = link->getInboundJoint();
                PxU32 offset = mEntries[i].dofStarts[j];
                if (joint)
                {
                    PxSpatialForce sf = cache->linkIncomingJointForce[j];
                    PxArticulationJointType::Enum jointType = joint->getJointType();
                    if (!mEntries[i].isIncomingJointBody0Parent[j])
                    {
                        transformToParentFrame(joint, sf.force, sf.torque);
                    }
                    switch (jointType)
                    {
                    case PxArticulationJointType::eUNDEFINED:
                        CARB_LOG_ERROR("Undefined joint type encountered while calculating forces along DOFs");
                        break;
                    case PxArticulationJointType::eFIX:
                        break;
                    case PxArticulationJointType::eREVOLUTE:
                        dst[offset] = sf.torque.x;
                        break;
                    case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
                        dst[offset] = sf.torque.x;
                        break;
                    case PxArticulationJointType::ePRISMATIC:
                        dst[offset] = sf.force.x;
                        break;
                    case PxArticulationJointType::eSPHERICAL:
                        PxQuat physxToUsdRotation = mEntries[i].incomingJointPhysxToUsdRotations[j];
                        CARB_ASSERT(physxToUsdRotation.isUnit());
                        PxVec3 usdSpaceTorque = physxToUsdRotation.rotate(sf.torque);
                        // printf("cpu eSPHERICAL offset = %u torque= %f,%f,%f usdSpaceTorque= %f,%f,%f \n", offset, sf.torque.x,
                        //        sf.torque.y, sf.torque.z, usdSpaceTorque.x, usdSpaceTorque.y, usdSpaceTorque.z);
                        PxU32 DofIdx = 0;
                        if(mEntries[i].freeD6Axes[j] & FreeD6RotationAxesFlag::eTWIST)
                            dst[offset + DofIdx++] = usdSpaceTorque.x;

                        if(mEntries[i].freeD6Axes[j] & FreeD6RotationAxesFlag::eSWING1)
                            dst[offset + DofIdx++] = usdSpaceTorque.y;

                        if(mEntries[i].freeD6Axes[j] & FreeD6RotationAxesFlag::eSWING2)
                            dst[offset + DofIdx++] = usdSpaceTorque.z;
                        break;
                    }
                }
            }
        }
    }

    return true;
}

void CpuArticulationView::prepareDirtyForceTracker()
{
    if (!mDirtyForceTracker)
    {
        mDirtyForceTracker = std::make_shared<CpuRigidBodyDirtyForceTracker>();

        PxU32 numBodies = getCount() * mMaxLinks;
        mDirtyForceTracker->bodies.resize(numBodies);
        mDirtyForceTracker->dirtyFlags.resize(numBodies);

        for (PxU32 i = 0; i < getCount(); i++)
        {
            for (PxU32 j = 0; j < mEntries[i].numLinks; j++)
            {
                mDirtyForceTracker->bodies[i * mMaxLinks + j] = mEntries[i].links[j];
            }
        }

        if (mCpuSimData)
        {
            mCpuSimData->addRigidBodyDirtyForceTracker(mDirtyForceTracker);
        }
    }
}

bool CpuArticulationView::applyForcesAndTorquesAtPosition(const TensorDesc* srcForceTensor,
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
                           checkTensorSizeExact(*srcForceTensor, getCount() * mMaxLinks * 3u, "force", __FUNCTION__);

    if (hasTorque)
        validTorqueTensor = checkTensorDevice(*srcTorqueTensor, -1, "torque", __FUNCTION__) &&
                            checkTensorFloat32(*srcTorqueTensor, "torque", __FUNCTION__) &&
                            checkTensorSizeExact(*srcTorqueTensor, getCount() * mMaxLinks * 3u, "torque", __FUNCTION__);

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
                              checkTensorSizeExact(*srcPositionTensor, getCount() * mMaxLinks * 3u, "position", __FUNCTION__);
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
            ArticulationEntry& entry = mEntries[idx];
            for (PxU32 j = 0; j < mEntries[i].numLinks; j++)
            {
                if (validForceTensor)
                {
                    const float* src = static_cast<const float*>(srcForceTensor->data) + (idx * mMaxLinks + j) * 3;
                    PxVec3 force(src[0], src[1], src[2]);
                    if (!isGlobal)
                    {
                        // translate force vector into global space
                        PxTransform pose = entry.links[j]->getGlobalPose();
                        force = pose.q.rotate(force);
                    }
                    // printf("CPU idx %u force = %f,%f,%f\n", idx, force.x, force.y, force.z);
                    entry.links[j]->addForce(force);
                    mDirtyForceTracker->dirtyFlags[idx * mMaxLinks + j] |= RigidBodyDirtyForceFlags::eForce;
                    if (validPositionTensor)
                    {
                        PxTransform pose = entry.links[j]->getGlobalPose();
                        const PxVec3 com = pose.transform(entry.links[j]->getCMassLocalPose().p);
                        const float* srcP = static_cast<const float*>(srcPositionTensor->data) + (idx * mMaxLinks + j) * 3;
                        PxVec3 position(srcP[0], srcP[1], srcP[2]);
                        if (!isGlobal)
                            position = pose.transform(position);
                        PxVec3 tmp = (position - com).cross(force);
                        // printf("CPU idx %u torque = %f,%f,%f\n", idx, tmp.x, tmp.y, tmp.z);
                        entry.links[j]->addTorque((position - com).cross(force));
                        mDirtyForceTracker->dirtyFlags[idx * mMaxLinks + j] |= RigidBodyDirtyForceFlags::eTorque;
                    }
                }
                if (validTorqueTensor)
                {
                    const float* src = static_cast<const float*>(srcTorqueTensor->data) + (idx * mMaxLinks + j) * 3;
                    PxVec3 torque;
                    torque.x = src[0];
                    torque.y = src[1];
                    torque.z = src[2];
                    if (!isGlobal)
                    {
                        // translate force vector into global space
                        PxTransform pose = entry.links[j]->getGlobalPose();
                        torque = pose.q.rotate(torque);
                    }
                    entry.links[j]->addTorque(torque);
                    mDirtyForceTracker->dirtyFlags[idx * mMaxLinks + j] |= RigidBodyDirtyForceFlags::eTorque;
                }
            }
        }
    }

    mDirtyForceTracker->isDirty = true;

    return true;
}

bool CpuArticulationView::getFixedTendonStiffnesses(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "Tendon stiffness", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "Tendon stiffness", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxFixedTendons, "Tendon stiffness", __FUNCTION__))
    {
        return false;
    }

    float* dst = (float*)dstTensor->data;
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        for (PxU32 j = 0; j < mEntries[i].numFixedTendons; j++)
        {
            *dst++ = mEntries[i].fixedTendons[j]->getStiffness();
        }
    }

    return true;
}

bool CpuArticulationView::getFixedTendonDampings(const TensorDesc* dstTensor) const
{
    PASS_EMPTY_TENSOR(dstTensor);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "Tendon damping", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "Tendon damping", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxFixedTendons, "Tendon damping", __FUNCTION__))
    {
        return false;
    }

    float* dst = (float*)dstTensor->data;
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        for (PxU32 j = 0; j < mEntries[i].numFixedTendons; j++)
        {
            *dst++ = mEntries[i].fixedTendons[j]->getDamping();
        }
    }

    return true;
}

bool CpuArticulationView::getFixedTendonLimitStiffnesses(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "Tendon limit stiffness", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "Tendon limit stiffness", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxFixedTendons, "Tendon limit stiffness", __FUNCTION__))
    {
        return false;
    }

    float* dst = (float*)dstTensor->data;
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        for (PxU32 j = 0; j < mEntries[i].numFixedTendons; j++)
        {
            *dst++ = mEntries[i].fixedTendons[j]->getLimitStiffness();
        }
    }

    return true;
}

bool CpuArticulationView::getFixedTendonLimits(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "Tendon limits", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "Tendon limits", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxFixedTendons * 2, "Tendon limits", __FUNCTION__))
    {
        return false;
    }

    float* dst = (float*)dstTensor->data;
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        for (PxU32 j = 0; j < mEntries[i].numFixedTendons; j++)
        {
            *dst++ = mEntries[i].fixedTendons[j]->getLimitParameters().lowLimit;
            *dst++ = mEntries[i].fixedTendons[j]->getLimitParameters().highLimit;
        }
    }

    return true;
}

bool CpuArticulationView::getFixedTendonfixedSpringRestLengths(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "Tendon rest length", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "Tendon rest length", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxFixedTendons, "Tendon rest length", __FUNCTION__))
    {
        return false;
    }

    float* dst = (float*)dstTensor->data;
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        for (PxU32 j = 0; j < mEntries[i].numFixedTendons; j++)
        {
            *dst++ = mEntries[i].fixedTendons[j]->getRestLength();
        }
    }

    return true;
}

bool CpuArticulationView::getFixedTendonOffsets(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "Tendon offset", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "Tendon offset", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxFixedTendons, "Tendon offset", __FUNCTION__))
    {
        return false;
    }

    float* dst = (float*)dstTensor->data;
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        for (PxU32 j = 0; j < mEntries[i].numFixedTendons; j++)
        {
            *dst++ = mEntries[i].fixedTendons[j]->getOffset();
        }
    }

    return true;
}

bool CpuArticulationView::getSpatialTendonStiffnesses(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "Tendon stiffness", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "Tendon stiffness", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxSpatialTendons, "Tendon stiffness", __FUNCTION__))
    {
        return false;
    }

    float* dst = (float*)dstTensor->data;
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        for (PxU32 j = 0; j < mEntries[i].numSpatialTendons; j++)
        {
            *dst++ = mEntries[i].spatialTendons[j]->getStiffness();
        }
    }

    return true;
}

bool CpuArticulationView::getSpatialTendonDampings(const TensorDesc* dstTensor) const
{
    PASS_EMPTY_TENSOR(dstTensor);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "Tendon damping", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "Tendon damping", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxSpatialTendons, "Tendon damping", __FUNCTION__))
    {
        return false;
    }

    float* dst = (float*)dstTensor->data;
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        for (PxU32 j = 0; j < mEntries[i].numSpatialTendons; j++)
        {
            *dst++ = mEntries[i].spatialTendons[j]->getDamping();
        }
    }

    return true;
}

bool CpuArticulationView::getSpatialTendonLimitStiffnesses(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "Tendon limit stiffness", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "Tendon limit stiffness", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxSpatialTendons, "Tendon limit stiffness", __FUNCTION__))
    {
        return false;
    }

    float* dst = (float*)dstTensor->data;
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        for (PxU32 j = 0; j < mEntries[i].numSpatialTendons; j++)
        {
            *dst++ = mEntries[i].spatialTendons[j]->getLimitStiffness();
        }
    }

    return true;
}

bool CpuArticulationView::getSpatialTendonOffsets(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "Tendon offset", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "Tendon offset", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxSpatialTendons, "Tendon offset", __FUNCTION__))
    {
        return false;
    }

    float* dst = (float*)dstTensor->data;
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        for (PxU32 j = 0; j < mEntries[i].numSpatialTendons; j++)
        {
            *dst++ = mEntries[i].spatialTendons[j]->getOffset();
        }
    }

    return true;
}

bool CpuArticulationView::setFixedTendonProperties(const TensorDesc* stiffnesses, const TensorDesc* dampings, const TensorDesc* limitStiffnesses, const TensorDesc* limits, const TensorDesc* restLengths, const TensorDesc* offsets, const TensorDesc* indexTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(stiffnesses);
    PASS_EMPTY_TENSOR(dampings);
    PASS_EMPTY_TENSOR(limitStiffnesses);
    PASS_EMPTY_TENSOR(limits);
    PASS_EMPTY_TENSOR(restLengths);
    PASS_EMPTY_TENSOR(offsets);

    if (!stiffnesses || !stiffnesses->data)
    {
        return false;
    }
    if (!dampings || !dampings->data)
    {
        return false;
    }
    if (!limitStiffnesses || !limitStiffnesses->data)
    {
        return false;
    }
    if (!limits || !limits->data)
    {
        return false;
    }
    if (!restLengths || !restLengths->data)
    {
        return false;
    }
    if (!offsets || !offsets->data)
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

    if (!checkTensorDevice(*stiffnesses, -1, "tendon stiffness", __FUNCTION__) ||
        !checkTensorFloat32(*stiffnesses, "tendon stiffness", __FUNCTION__) ||
        !checkTensorSizeExact(*stiffnesses, getCount() * mMaxFixedTendons, "tendon stiffness", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            const float* src = static_cast<const float*>(stiffnesses->data) + idx * mMaxFixedTendons;
            for (PxU32 j = 0; j < mEntries[idx].numFixedTendons; j++)
            {
                mEntries[idx].fixedTendons[j]->setStiffness(src[j]);
            }
        }
    }

    if (!checkTensorDevice(*dampings, -1, "tendon damping", __FUNCTION__) ||
        !checkTensorFloat32(*dampings, "tendon damping", __FUNCTION__) ||
        !checkTensorSizeExact(*dampings, getCount() * mMaxFixedTendons, "tendon damping", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            const float* src = static_cast<const float*>(dampings->data) + idx * mMaxFixedTendons;
            for (PxU32 j = 0; j < mEntries[idx].numFixedTendons; j++)
            {
                mEntries[idx].fixedTendons[j]->setDamping(src[j]);
            }
        }
    }

    if (!checkTensorDevice(*limitStiffnesses, -1, "tendon limit stiffness", __FUNCTION__) ||
        !checkTensorFloat32(*limitStiffnesses, "tendon limit stiffness", __FUNCTION__) ||
        !checkTensorSizeExact(*limitStiffnesses, getCount() * mMaxFixedTendons, "tendon limit stiffness", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            const float* src = static_cast<const float*>(limitStiffnesses->data) + idx * mMaxFixedTendons;
            for (PxU32 j = 0; j < mEntries[idx].numFixedTendons; j++)
            {
                mEntries[idx].fixedTendons[j]->setLimitStiffness(src[j]);
            }
        }
    }

    if (!checkTensorDevice(*limits, -1, "tendon limits", __FUNCTION__) ||
        !checkTensorFloat32(*limits, "tendon limits", __FUNCTION__) ||
        !checkTensorSizeExact(*limits, getCount() * mMaxFixedTendons * 2u, "tendon limits", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            const float* src = static_cast<const float*>(limits->data) + idx * mMaxFixedTendons * 2;
            for (PxU32 j = 0; j < mEntries[idx].numFixedTendons; j++)
            {
                PxArticulationTendonLimit limit;
                limit.lowLimit = src[j*2];
                limit.highLimit = src[j*2+1];
                mEntries[idx].fixedTendons[j]->setLimitParameters(limit);
            }
        }
    }

    if (!checkTensorDevice(*restLengths, -1, "tendon rest length", __FUNCTION__) ||
        !checkTensorFloat32(*restLengths, "tendon rest length", __FUNCTION__) ||
        !checkTensorSizeExact(*restLengths, getCount() * mMaxFixedTendons, "tendon rest length", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            const float* src = static_cast<const float*>(restLengths->data) + idx * mMaxFixedTendons;
            for (PxU32 j = 0; j < mEntries[idx].numFixedTendons; j++)
            {
                mEntries[idx].fixedTendons[j]->setRestLength(src[j]);
            }
        }
    }

    if (!checkTensorDevice(*offsets, -1, "tendon offset", __FUNCTION__) ||
        !checkTensorFloat32(*offsets, "tendon offset", __FUNCTION__) ||
        !checkTensorSizeExact(*offsets, getCount() * mMaxFixedTendons, "tendon offset", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            const float* src = static_cast<const float*>(offsets->data) + idx * mMaxFixedTendons;
            for (PxU32 j = 0; j < mEntries[idx].numFixedTendons; j++)
            {
                mEntries[idx].fixedTendons[j]->setOffset(src[j]);
            }
        }
    }

    return true;
}

bool CpuArticulationView::setSpatialTendonProperties(const TensorDesc* stiffnesses, const TensorDesc* dampings, const TensorDesc* limitStiffnesses, const TensorDesc* offsets, const TensorDesc* indexTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(stiffnesses);
    PASS_EMPTY_TENSOR(dampings);
    PASS_EMPTY_TENSOR(limitStiffnesses);
    PASS_EMPTY_TENSOR(offsets);

    if (!stiffnesses || !stiffnesses->data)
    {
        return false;
    }
    if (!dampings || !dampings->data)
    {
        return false;
    }
    if (!limitStiffnesses || !limitStiffnesses->data)
    {
        return false;
    }
    if (!offsets || !offsets->data)
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

    if (!checkTensorDevice(*stiffnesses, -1, "tendon stiffness", __FUNCTION__) ||
        !checkTensorFloat32(*stiffnesses, "tendon stiffness", __FUNCTION__) ||
        !checkTensorSizeExact(*stiffnesses, getCount() * mMaxSpatialTendons, "tendon stiffness", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            const float* src = static_cast<const float*>(stiffnesses->data) + idx * mMaxSpatialTendons;
            for (PxU32 j = 0; j < mEntries[idx].numSpatialTendons; j++)
            {
                mEntries[idx].spatialTendons[j]->setStiffness(src[j]);
            }
        }
    }

    if (!checkTensorDevice(*dampings, -1, "tendon damping", __FUNCTION__) ||
        !checkTensorFloat32(*dampings, "tendon damping", __FUNCTION__) ||
        !checkTensorSizeExact(*dampings, getCount() * mMaxSpatialTendons, "tendon damping", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            const float* src = static_cast<const float*>(dampings->data) + idx * mMaxSpatialTendons;
            for (PxU32 j = 0; j < mEntries[idx].numSpatialTendons; j++)
            {
                mEntries[idx].spatialTendons[j]->setDamping(src[j]);
            }
        }
    }

    if (!checkTensorDevice(*limitStiffnesses, -1, "tendon limit stiffness", __FUNCTION__) ||
        !checkTensorFloat32(*limitStiffnesses, "tendon limit stiffness", __FUNCTION__) ||
        !checkTensorSizeExact(*limitStiffnesses, getCount() * mMaxSpatialTendons, "tendon limit stiffness", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            const float* src = static_cast<const float*>(limitStiffnesses->data) + idx * mMaxSpatialTendons;
            for (PxU32 j = 0; j < mEntries[idx].numSpatialTendons; j++)
            {
                mEntries[idx].spatialTendons[j]->setLimitStiffness(src[j]);
            }
        }
    }

    if (!checkTensorDevice(*offsets, -1, "tendon offset", __FUNCTION__) ||
        !checkTensorFloat32(*offsets, "tendon offset", __FUNCTION__) ||
        !checkTensorSizeExact(*offsets, getCount() * mMaxSpatialTendons, "tendon offset", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            const float* src = static_cast<const float*>(offsets->data) + idx * mMaxSpatialTendons;
            for (PxU32 j = 0; j < mEntries[idx].numSpatialTendons; j++)
            {
                mEntries[idx].spatialTendons[j]->setOffset(src[j]);
            }
        }
    }

    return true;
}

}
}
}
