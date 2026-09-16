// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-TENSOR-CONTACT-001
 * @covers AC-1 AC-2 AC-3 AC-4
 *
 * @implements REQ-PUBLICAPI-001
 * @covers AC-40
 */

#include "tensors/cpu/CpuRigidContactView.h"
#include "tensors/cpu/CpuSimulationView.h"

#include "tensors/GlobalsAreBad.h"
#include "tensors/CommonTypes.h"

#include <PxPhysicsAPI.h>

#include <carb/logging/Log.h>
#include <omni/physx/IPhysx.h>

#include <omni/physics/tensors/TensorUtils.h>

#include <utility>

using omni::physics::tensors::checkTensorDevice;
using omni::physics::tensors::checkTensorFloat32;
using omni::physics::tensors::checkTensorInt32;
using omni::physics::tensors::checkTensorInt64;
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

CpuRigidContactView::CpuRigidContactView(CpuSimulationView* sim,
                                         std::vector<RigidContactSensorEntry>&& entries,
                                         uint32_t numFilters,
                                         uint32_t maxContactDataCount)
    : BaseRigidContactView(sim, std::move(entries), numFilters, maxContactDataCount)
{
    mCpuSimData = sim->getCpuSimulationData();

    PxU32 numSensors = PxU32(mEntries.size());

    mBuckets.resize(numSensors);

    if (mCpuSimData)
    {
        for (PxU32 i = 0; i < numSensors; i++)
        {
            mCpuSimData->addRigidContactBucket(mEntries[i].referentId, &mBuckets[i]);
        }
    }
}

CpuRigidContactView::~CpuRigidContactView()
{
    if (mCpuSimData)
    {
        PxU32 numSensors = PxU32(mEntries.size());
        for (PxU32 i = 0; i < numSensors; i++)
        {
            mCpuSimData->removeRigidContactBucket(mEntries[i].referentId, &mBuckets[i]);
        }
    }
}

bool CpuRigidContactView::getNetContactForces(const TensorDesc* dstTensor, float dt) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    float* dstForce = nullptr;
    if (dstTensor && dstTensor->data)
    {
        if (!checkTensorDevice(*dstTensor, -1, "net contact forces", __FUNCTION__) ||
            !checkTensorFloat32(*dstTensor, "net contact forces", __FUNCTION__) ||
            !checkTensorSizeExact(*dstTensor, getSensorCount() * 3, "net contact forces", __FUNCTION__))
        {
            return false;
        }
        dstForce = static_cast<float*>(dstTensor->data);
    }

    // make sure we have the latest contact reports
    mCpuSimData->updateContactReports();

    float invDt = 1.0f / dt;

    const ::omni::physx::ContactData* globalContactData = mCpuSimData->getCurrentContactData();
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        PxVec3 netImpulse(0.0f);
        
        uint32_t headerCount = mBuckets[i].getHeaderCount();
        for (PxU32 j = 0; j < headerCount; j++)
        {
            const RigidContactHeaderRef& headerRef = mBuckets[i].getHeaderRef(j);
            const ::omni::physx::ContactEventHeader* header = headerRef.header;
            const ::omni::physx::ContactData* contactData = globalContactData + header->contactDataOffset;
            for (PxU32 k = 0; k < header->numContactData; k++)
            {
                const ::omni::physx::ContactData& cdata = contactData[k];
                if (!headerRef.invert)
                {
                    netImpulse.x += cdata.impulse.x;
                    netImpulse.y += cdata.impulse.y;
                    netImpulse.z += cdata.impulse.z;
                }
                else
                {
                    netImpulse.x -= cdata.impulse.x;
                    netImpulse.y -= cdata.impulse.y;
                    netImpulse.z -= cdata.impulse.z;
                }
            }
        }

        // assumes that all contacts had the same dt
        if (dstForce)
        {
            *dstForce++ = invDt * netImpulse.x;
            *dstForce++ = invDt * netImpulse.y;
            *dstForce++ = invDt * netImpulse.z;
        }
    }

    return true;
}

bool CpuRigidContactView::getContactForceMatrix(const TensorDesc* dstTensor, float dt) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "contact force matrix", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "contact force matrix", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getSensorCount() * getFilterCount() * 3, "contact force matrix", __FUNCTION__))
    {
        return false;
    }

    if (mCpuSimData)
    {
        // make sure we have the latest contact reports
        mCpuSimData->updateContactReports();

        float invDt = 1.0f / dt;

        float* dst = static_cast<float*>(dstTensor->data);
        const ::omni::physx::ContactData* globalContactData = mCpuSimData->getCurrentContactData();

        uint32_t numSensors = getSensorCount();
        uint32_t numFilters = getFilterCount();

        std::vector<PxVec3> netImpulses(numFilters);

        for (PxU32 i = 0; i < numSensors; i++)
        {
            for (PxU32 k = 0; k < numFilters; k++)
            {
                netImpulses[k] = {0.0f, 0.0f, 0.0f};
            }

            uint32_t headerCount = mBuckets[i].getHeaderCount();
            for (PxU32 j = 0; j < headerCount; j++)
            {
                const RigidContactHeaderRef& headerRef = mBuckets[i].getHeaderRef(j);
                const ::omni::physx::ContactEventHeader* header = headerRef.header;
                auto& filterIndexMap = mEntries[i].filterIndexMap;

                uint64_t otherActor, otherCollider;
                if (!headerRef.invert)
                {
                    otherActor = header->actor1;
                    otherCollider = header->collider1;
                }
                else
                {
                    otherActor = header->actor0;
                    otherCollider = header->collider0;
                }

                auto indexIter = filterIndexMap.find(otherActor);
                if (indexIter == filterIndexMap.end())
                {
                    if (otherCollider != otherActor)
                    {
                        indexIter = filterIndexMap.find(otherCollider);
                    }
                }

                if (indexIter != filterIndexMap.end())
                {
                    uint32_t idx = indexIter->second;
                    const ::omni::physx::ContactData* contactData = globalContactData + header->contactDataOffset;
                    for (PxU32 k = 0; k < header->numContactData; k++)
                    {
                        const ::omni::physx::ContactData& cdata = contactData[k];
                        if (!headerRef.invert)
                        {
                            netImpulses[idx].x += cdata.impulse.x;
                            netImpulses[idx].y += cdata.impulse.y;
                            netImpulses[idx].z += cdata.impulse.z;
                        }
                        else
                        {
                            netImpulses[idx].x -= cdata.impulse.x;
                            netImpulses[idx].y -= cdata.impulse.y;
                            netImpulses[idx].z -= cdata.impulse.z;
                        }
                    }
                }
            }

            for (PxU32 k = 0; k < numFilters; k++)
            {
                // assumes that all contacts had the same dt
                *dst++ = invDt * netImpulses[k].x;
                *dst++ = invDt * netImpulses[k].y;
                *dst++ = invDt * netImpulses[k].z;
            }
        }
    }

    return true;
}

bool CpuRigidContactView::getContactData(const TensorDesc* contactForceTensor,
                                         const TensorDesc* contactPointTensor,
                                         const TensorDesc* contactNormalTensor,
                                         const TensorDesc* contactSeparationTensor,
                                         const TensorDesc* contactCountTensor,
                                         const TensorDesc* contactStartIndicesTensor,
                                         float dt) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);

    if (!contactForceTensor || !contactForceTensor->data || !contactPointTensor || !contactPointTensor->data ||
        !contactNormalTensor || !contactNormalTensor->data || !contactSeparationTensor ||
        !contactSeparationTensor->data || !contactCountTensor || !contactCountTensor->data ||
        !contactStartIndicesTensor || !contactStartIndicesTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*contactForceTensor, -1, "contact force buffer", __FUNCTION__) ||
        !checkTensorFloat32(*contactForceTensor, "contact force buffer", __FUNCTION__) ||
        !checkTensorSizeExact(*contactForceTensor, getMaxContactDataCount(), "contact force buffer", __FUNCTION__))
    {
        return false;
    }
    
    if (!checkTensorDevice(*contactPointTensor, -1, "contact point buffer", __FUNCTION__) ||
        !checkTensorFloat32(*contactPointTensor, "contact point buffer", __FUNCTION__) ||
        !checkTensorSizeExact(*contactPointTensor, getMaxContactDataCount() * 3, "contact point buffer", __FUNCTION__))
    {
        return false;
    }
    
    if (!checkTensorDevice(*contactNormalTensor, -1, "contact normal buffer", __FUNCTION__) ||
        !checkTensorFloat32(*contactNormalTensor, "contact normal buffer", __FUNCTION__) ||
        !checkTensorSizeExact(*contactNormalTensor, getMaxContactDataCount() * 3, "contact normal buffer", __FUNCTION__))
    {
        return false;
    }

    if (!checkTensorDevice(*contactSeparationTensor, -1, "contact separation buffer", __FUNCTION__) ||
        !checkTensorFloat32(*contactSeparationTensor, "contact separation buffer", __FUNCTION__) ||
        !checkTensorSizeExact(*contactSeparationTensor, getMaxContactDataCount(), "contact separation buffer", __FUNCTION__))
    {
        return false;
    }

    if (!checkTensorDevice(*contactCountTensor, -1, "contact count buffer", __FUNCTION__) ||
        !checkTensorInt32(*contactCountTensor, "contact count buffer", __FUNCTION__) ||
        !checkTensorSizeExact(*contactCountTensor, getSensorCount() * getFilterCount() , "contact count buffer", __FUNCTION__))
    {
        return false;
    }

    if (!checkTensorDevice(*contactStartIndicesTensor, -1, "contact start indices buffer", __FUNCTION__) ||
        !checkTensorInt32(*contactStartIndicesTensor, "contact start indices buffer", __FUNCTION__) ||
        !checkTensorSizeExact(*contactStartIndicesTensor, getSensorCount() * getFilterCount() , "contact start indices buffer", __FUNCTION__))
    {
        return false;
    }


    if (mCpuSimData)
    {
        // make sure we have the latest contact reports
        mCpuSimData->updateContactReports();
        float timeStepInv = 1.0f / dt;
        PxReal* dstForces = static_cast<PxReal*>(contactForceTensor->data);
        PxVec3* dstPoints = static_cast<PxVec3*>(contactPointTensor->data);
        PxVec3* dstNormals = static_cast<PxVec3*>(contactNormalTensor->data);
        PxReal* dstSeparations = static_cast<PxReal*>(contactSeparationTensor->data);
        PxU32* dstContactCount= static_cast<PxU32*>(contactCountTensor->data);
        PxU32* dstStartIndices = static_cast<PxU32*>(contactStartIndicesTensor->data);

        memset(dstForces, 0, getMaxContactDataCount() * sizeof(PxReal));
        memset(dstPoints, 0, getMaxContactDataCount() * sizeof(PxVec3));
        memset(dstNormals, 0, getMaxContactDataCount() * sizeof(PxVec3));
        memset(dstSeparations, 0, getMaxContactDataCount() * sizeof(PxReal));
        memset(dstContactCount, 0, getSensorCount() * getFilterCount() * sizeof(PxU32));
        memset(dstStartIndices, 0, getSensorCount() * getFilterCount() * sizeof(PxU32));

        const ::omni::physx::ContactData* globalContactData = mCpuSimData->getCurrentContactData();
        uint32_t numSensors = getSensorCount();
        uint32_t numFilters = getFilterCount();
        // contact counting part
        for (PxU32 i = 0; i < numSensors; i++)
        {
            uint32_t headerCount = mBuckets[i].getHeaderCount();
            for (PxU32 j = 0; j < headerCount; j++)
            {
                const RigidContactHeaderRef& headerRef = mBuckets[i].getHeaderRef(j);
                const ::omni::physx::ContactEventHeader* header = headerRef.header;
                auto& filterIndexMap = mEntries[i].filterIndexMap;

                uint64_t otherActor, otherCollider;
                if (!headerRef.invert)
                {
                    otherActor = header->actor1;
                    otherCollider = header->collider1;
                }
                else
                {
                    otherActor = header->actor0;
                    otherCollider = header->collider0;
                }

                auto indexIter = filterIndexMap.find(otherActor);
                if (indexIter == filterIndexMap.end())
                {
                    if (otherCollider != otherActor)
                    {
                        indexIter = filterIndexMap.find(otherCollider);
                    }
                }

                if (indexIter != filterIndexMap.end())
                {
                    uint32_t idx = indexIter->second;
                    const ::omni::physx::ContactData* contactData = globalContactData + header->contactDataOffset;
                    for (PxU32 k = 0; k < header->numContactData; k++)
                    {
                        dstContactCount[i * numFilters + idx]++;
                    }
                }
            }
        }
        // prefix scan
        for (PxU32 i = 0; i < numSensors; i++)
        {
            for (PxU32 j = 0; j < numFilters; j++)
            {
                if (i != 0 || j > 0)
                    dstStartIndices[i * numFilters + j] +=
                        dstStartIndices[i * numFilters + j - 1] + dstContactCount[i * numFilters + j - 1];
            }
        }

        PxU32 totalCount = dstStartIndices[numSensors * numFilters - 1] + dstContactCount[numSensors * numFilters - 1];
        if (totalCount > getMaxContactDataCount())
            CARB_LOG_WARN(
                "Incomplete contact data is reported in CpuRigidContactView::getContactData because there are more contact data points than specified maxContactDataCount = %u.",
                getMaxContactDataCount());


        memset(dstContactCount, 0, getSensorCount() * getFilterCount() * sizeof(PxU32));

        for (PxU32 i = 0; i < numSensors; i++)
        {
            uint32_t headerCount = mBuckets[i].getHeaderCount();
            for (PxU32 j = 0; j < headerCount; j++)
            {
                const RigidContactHeaderRef& headerRef = mBuckets[i].getHeaderRef(j);
                const ::omni::physx::ContactEventHeader* header = headerRef.header;
                auto& filterIndexMap = mEntries[i].filterIndexMap;

                uint64_t otherActor, otherCollider;
                if (!headerRef.invert)
                {
                    otherActor = header->actor1;
                    otherCollider = header->collider1;
                }
                else
                {
                    otherActor = header->actor0;
                    otherCollider = header->collider0;
                }

                auto indexIter = filterIndexMap.find(otherActor);
                if (indexIter == filterIndexMap.end())
                {
                    if (otherCollider != otherActor)
                    {
                        indexIter = filterIndexMap.find(otherCollider);
                    }
                }

                if (indexIter != filterIndexMap.end())
                {
                    uint32_t idx = indexIter->second;
                    const ::omni::physx::ContactData* contactData = globalContactData + header->contactDataOffset;
                    for (PxU32 k = 0; k < header->numContactData; k++)
                    {
                        const ::omni::physx::ContactData& cdata = contactData[k];
                        PxU32 currentCount = dstContactCount[i * numFilters + idx]++;
                        PxU32 elementIdx = dstStartIndices[i * numFilters + idx] + currentCount;

                        if (elementIdx < getMaxContactDataCount())
                        {
                            if (!headerRef.invert)
                            {
                                dstForces[elementIdx] =
                                    PxVec3(cdata.impulse.x, cdata.impulse.y, cdata.impulse.z).magnitude() * timeStepInv;
                            }
                            else
                            {
                                dstForces[elementIdx] =
                                    -PxVec3(cdata.impulse.x, cdata.impulse.y, cdata.impulse.z).magnitude() * timeStepInv;
                            }
                            dstNormals[elementIdx] = PxVec3(cdata.normal.x, cdata.normal.y, cdata.normal.z);
                            dstPoints[elementIdx] = PxVec3(cdata.position.x, cdata.position.y, cdata.position.z);
                            dstSeparations[elementIdx] = cdata.separation;
                        }
                    }
                }
            }
        }
    }

    return true;
}

bool CpuRigidContactView::getFrictionData(const TensorDesc* FrictionForceTensor,
                                          const TensorDesc* contactPointTensor,
                                          const TensorDesc* contactCountTensor,
                                          const TensorDesc* contactStartIndicesTensor,
                                          float dt) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    if (!FrictionForceTensor || !FrictionForceTensor->data || !contactPointTensor || !contactPointTensor->data ||
        !contactCountTensor || !contactCountTensor->data || !contactStartIndicesTensor ||
        !contactStartIndicesTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*FrictionForceTensor, -1, "friction force buffer", __FUNCTION__) ||
        !checkTensorFloat32(*FrictionForceTensor, "friction force buffer", __FUNCTION__) ||
        !checkTensorSizeExact(*FrictionForceTensor, getMaxContactDataCount() * 3, "friction force buffer", __FUNCTION__))
    {
        return false;
    }
    if (!checkTensorDevice(*contactPointTensor, -1, "contact point buffer", __FUNCTION__) ||
        !checkTensorFloat32(*contactPointTensor, "contact point buffer", __FUNCTION__) ||
        !checkTensorSizeExact(*contactPointTensor, getMaxContactDataCount() * 3, "contact point buffer", __FUNCTION__))
    {
        return false;
    }

    if (!checkTensorDevice(*contactCountTensor, -1, "contact count matrix", __FUNCTION__) ||
        !checkTensorInt32(*contactCountTensor, "contact count matrix", __FUNCTION__) ||
        !checkTensorSizeExact(
            *contactCountTensor, getSensorCount() * getFilterCount(), "contact count matrix", __FUNCTION__))
    {
        return false;
    }
    if (!checkTensorDevice(*contactStartIndicesTensor, -1, "contact start indices matrix", __FUNCTION__) ||
        !checkTensorInt32(*contactStartIndicesTensor, "contact start indices matrix", __FUNCTION__) ||
        !checkTensorSizeExact(*contactStartIndicesTensor, getSensorCount() * getFilterCount(),
                              "contact start indices matrix", __FUNCTION__))
    {
        return false;
    }


    if (mCpuSimData)
    {
        // make sure we have the latest contact reports
        mCpuSimData->updateContactReports();
        float timeStepInv = 1.0f / dt;
        PxVec3* dstForces = static_cast<PxVec3*>(FrictionForceTensor->data);
        PxVec3* dstPoints = static_cast<PxVec3*>(contactPointTensor->data);
        PxU32* dstCounts = static_cast<PxU32*>(contactCountTensor->data);
        PxU32* dstStartIndices = static_cast<PxU32*>(contactStartIndicesTensor->data);

        memset(dstForces, 0, getMaxContactDataCount() * sizeof(PxVec3));
        memset(dstPoints, 0, getMaxContactDataCount() * sizeof(PxVec3));
        memset(dstCounts, 0, getSensorCount() * getFilterCount() * sizeof(PxU32));
        memset(dstStartIndices, 0, getSensorCount() * getFilterCount() * sizeof(PxU32));

        const ::omni::physx::FrictionAnchor* globalFrictionData = mCpuSimData->getCurrentFrictionData();
        uint32_t numSensors = getSensorCount();
        uint32_t numFilters = getFilterCount();
        // contact counting part
        for (PxU32 i = 0; i < numSensors; i++)
        {
            uint32_t headerCount = mBuckets[i].getHeaderCount();
            for (PxU32 j = 0; j < headerCount; j++)
            {
                const RigidContactHeaderRef& headerRef = mBuckets[i].getHeaderRef(j);
                const ::omni::physx::ContactEventHeader* header = headerRef.header;
                auto& filterIndexMap = mEntries[i].filterIndexMap;

                uint64_t otherActor, otherCollider;
                if (!headerRef.invert)
                {
                    otherActor = header->actor1;
                    otherCollider = header->collider1;
                }
                else
                {
                    otherActor = header->actor0;
                    otherCollider = header->collider0;
                }

                auto indexIter = filterIndexMap.find(otherActor);
                if (indexIter == filterIndexMap.end())
                {
                    if (otherCollider != otherActor)
                    {
                        indexIter = filterIndexMap.find(otherCollider);
                    }
                }

                if (indexIter != filterIndexMap.end())
                {
                    uint32_t idx = indexIter->second;
                    const ::omni::physx::FrictionAnchor* frictionData = globalFrictionData + header->frictionAnchorsDataOffset;
                    for (PxU32 k = 0; k < header->numfrictionAnchorsData; k++)
                    {
                        dstCounts[i * numFilters + idx]++;
                    }
                }
            }
        }
        // prefix scan
        for (PxU32 i = 0; i < numSensors; i++)
        {
            for (PxU32 j = 0; j < numFilters; j++)
            {
                if (i != 0 || j > 0)
                    dstStartIndices[i * numFilters + j] +=
                        dstStartIndices[i * numFilters + j - 1] + dstCounts[i * numFilters + j - 1];
            }
        }

        PxU32 totalCount = dstStartIndices[numSensors * numFilters - 1] + dstCounts[numSensors * numFilters - 1];
        if (totalCount > getMaxContactDataCount())
            CARB_LOG_WARN(
                "Incomplete contact data is reported in CpuRigidContactView::getContactData because there are more contact data points than specified maxContactDataCount = %u.",
                getMaxContactDataCount());


        memset(dstCounts, 0, getSensorCount() * getFilterCount() * sizeof(PxU32));

        for (PxU32 i = 0; i < numSensors; i++)
        {
            uint32_t headerCount = mBuckets[i].getHeaderCount();
            for (PxU32 j = 0; j < headerCount; j++)
            {
                const RigidContactHeaderRef& headerRef = mBuckets[i].getHeaderRef(j);
                const ::omni::physx::ContactEventHeader* header = headerRef.header;
                auto& filterIndexMap = mEntries[i].filterIndexMap;

                uint64_t otherActor, otherCollider;
                if (!headerRef.invert)
                {
                    otherActor = header->actor1;
                    otherCollider = header->collider1;
                }
                else
                {
                    otherActor = header->actor0;
                    otherCollider = header->collider0;
                }

                auto indexIter = filterIndexMap.find(otherActor);
                if (indexIter == filterIndexMap.end())
                {
                    if (otherCollider != otherActor)
                    {
                        indexIter = filterIndexMap.find(otherCollider);
                    }
                }

                if (indexIter != filterIndexMap.end())
                {
                    uint32_t idx = indexIter->second;
                    const ::omni::physx::FrictionAnchor* frictionData = globalFrictionData + header->frictionAnchorsDataOffset;
                    for (PxU32 k = 0; k < header->numfrictionAnchorsData; k++)
                    {
                        const ::omni::physx::FrictionAnchor& fdata = frictionData[k];
                        PxU32 currentCount = dstCounts[i * numFilters + idx]++;
                        PxU32 elementIdx = dstStartIndices[i * numFilters + idx] + currentCount;

                        if (elementIdx < getMaxContactDataCount())
                        {
                            if (!headerRef.invert)
                            {
                                dstForces[elementIdx] =
                                    PxVec3(fdata.impulse.x, fdata.impulse.y, fdata.impulse.z) * timeStepInv;
                            }
                            else
                            {
                                dstForces[elementIdx] =
                                    -PxVec3(fdata.impulse.x, fdata.impulse.y, fdata.impulse.z) * timeStepInv;
                            }
                            dstPoints[elementIdx] = PxVec3(fdata.position.x, fdata.position.y, fdata.position.z);
                        }
                    }
                }
            }
        }
    }

    return true;
};

bool CpuRigidContactView::getRawContactData(const TensorDesc* contactForceTensor,
                                            const TensorDesc* contactPointTensor,
                                            const TensorDesc* contactNormalTensor,
                                            const TensorDesc* contactSeparationTensor,
                                            const TensorDesc* sensorLayoutTensor,
                                            const TensorDesc* actorIdsTensor,
                                            float dt) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);

    if (!contactForceTensor || !contactForceTensor->data || !contactPointTensor || !contactPointTensor->data ||
        !contactNormalTensor || !contactNormalTensor->data || !contactSeparationTensor ||
        !contactSeparationTensor->data || !sensorLayoutTensor || !sensorLayoutTensor->data ||
        !actorIdsTensor || !actorIdsTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*contactForceTensor, -1, "contact force buffer", __FUNCTION__) ||
        !checkTensorFloat32(*contactForceTensor, "contact force buffer", __FUNCTION__) ||
        !checkTensorSizeExact(*contactForceTensor, getMaxContactDataCount(), "contact force buffer", __FUNCTION__))
    {
        return false;
    }

    if (!checkTensorDevice(*contactPointTensor, -1, "contact point buffer", __FUNCTION__) ||
        !checkTensorFloat32(*contactPointTensor, "contact point buffer", __FUNCTION__) ||
        !checkTensorSizeExact(*contactPointTensor, getMaxContactDataCount() * 3, "contact point buffer", __FUNCTION__))
    {
        return false;
    }

    if (!checkTensorDevice(*contactNormalTensor, -1, "contact normal buffer", __FUNCTION__) ||
        !checkTensorFloat32(*contactNormalTensor, "contact normal buffer", __FUNCTION__) ||
        !checkTensorSizeExact(*contactNormalTensor, getMaxContactDataCount() * 3, "contact normal buffer", __FUNCTION__))
    {
        return false;
    }

    if (!checkTensorDevice(*contactSeparationTensor, -1, "contact separation buffer", __FUNCTION__) ||
        !checkTensorFloat32(*contactSeparationTensor, "contact separation buffer", __FUNCTION__) ||
        !checkTensorSizeExact(*contactSeparationTensor, getMaxContactDataCount(), "contact separation buffer", __FUNCTION__))
    {
        return false;
    }

    // Per-sensor layout is (numSensors, 2): column 0 count, column 1 start index. No
    // filter dimension.
    if (!checkTensorDevice(*sensorLayoutTensor, -1, "sensor layout buffer", __FUNCTION__) ||
        !checkTensorInt32(*sensorLayoutTensor, "sensor layout buffer", __FUNCTION__) ||
        !checkTensorSizeExact(*sensorLayoutTensor, getSensorCount() * 2, "sensor layout buffer", __FUNCTION__))
    {
        return false;
    }

    // Per-contact identities are (maxContactDataCount, 2): column 0 the reporting
    // sensor's actor, column 1 the actor it contacted.
    if (!checkTensorDevice(*actorIdsTensor, -1, "actor IDs buffer", __FUNCTION__) ||
        !checkTensorInt64(*actorIdsTensor, "actor IDs buffer", __FUNCTION__) ||
        !checkTensorSizeExact(*actorIdsTensor, getMaxContactDataCount() * 2, "actor IDs buffer", __FUNCTION__))
    {
        return false;
    }

    if (mCpuSimData)
    {
        mCpuSimData->updateContactReports();

        float timeStepInv = 1.0f / dt;
        PxReal* dstForces = static_cast<PxReal*>(contactForceTensor->data);
        PxVec3* dstPoints = static_cast<PxVec3*>(contactPointTensor->data);
        PxVec3* dstNormals = static_cast<PxVec3*>(contactNormalTensor->data);
        PxReal* dstSeparations = static_cast<PxReal*>(contactSeparationTensor->data);
        PxU32* dstSensorLayout = static_cast<PxU32*>(sensorLayoutTensor->data);
        uint64_t* dstActorIds = static_cast<uint64_t*>(actorIdsTensor->data);

        // The two-pass fill below indexes counts and starts per sensor many times over.
        // Run it on contiguous scratch and interleave into the caller's (S, 2) tensor at
        // the end, rather than threading a stride through every access -- numSensors is
        // small, and stride-2 arithmetic in the hot loops is where an off-by-one hides.
        std::vector<PxU32> sensorCounts(getSensorCount(), 0);
        std::vector<PxU32> sensorStarts(getSensorCount(), 0);
        PxU32* dstContactCount = sensorCounts.data();
        PxU32* dstStartIndices = sensorStarts.data();

        memset(dstForces, 0, getMaxContactDataCount() * sizeof(PxReal));
        memset(dstPoints, 0, getMaxContactDataCount() * sizeof(PxVec3));
        memset(dstNormals, 0, getMaxContactDataCount() * sizeof(PxVec3));
        memset(dstSeparations, 0, getMaxContactDataCount() * sizeof(PxReal));
        memset(dstSensorLayout, 0, getSensorCount() * 2 * sizeof(PxU32));
        memset(dstActorIds, 0, getMaxContactDataCount() * 2 * sizeof(uint64_t));

        const ::omni::physx::ContactData* globalContactData = mCpuSimData->getCurrentContactData();
        uint32_t numSensors = getSensorCount();

        // First pass: count contacts per sensor
        for (PxU32 i = 0; i < numSensors; i++)
        {
            uint32_t headerCount = mBuckets[i].getHeaderCount();
            for (PxU32 j = 0; j < headerCount; j++)
            {
                const RigidContactHeaderRef& headerRef = mBuckets[i].getHeaderRef(j);
                const ::omni::physx::ContactEventHeader* header = headerRef.header;
                dstContactCount[i] += header->numContactData;
            }
        }

        // Prefix scan to compute start indices
        for (PxU32 i = 1; i < numSensors; i++)
        {
            dstStartIndices[i] = dstStartIndices[i - 1] + dstContactCount[i - 1];
        }

        PxU32 totalCount = (numSensors > 0) ? (dstStartIndices[numSensors - 1] + dstContactCount[numSensors - 1]) : 0;
        if (totalCount > getMaxContactDataCount())
        {
            CARB_LOG_WARN(
                "Incomplete raw contact data: %u contacts found but maxContactDataCount = %u.",
                totalCount, getMaxContactDataCount());
        }

        // Reset contact counts for second pass
        memset(dstContactCount, 0, numSensors * sizeof(PxU32));

        // Second pass: fill contact data and both actor IDs. The header's actor0/actor1 are
        // already the legacy ObjectKey-handle ids the rest of the tensor API uses (see
        // ContactReport.cpp's keyToLegacyPathInt and CommonTypes.h's asInt()), so they are
        // written through unconverted; getOtherActorPathsFromIds resolves them.
        for (PxU32 i = 0; i < numSensors; i++)
        {
            uint32_t headerCount = mBuckets[i].getHeaderCount();
            for (PxU32 j = 0; j < headerCount; j++)
            {
                const RigidContactHeaderRef& headerRef = mBuckets[i].getHeaderRef(j);
                const ::omni::physx::ContactEventHeader* header = headerRef.header;
                const ::omni::physx::ContactData* contactData = globalContactData + header->contactDataOffset;

                uint64_t sensorActor = headerRef.invert ? header->actor1 : header->actor0;
                uint64_t otherActor = headerRef.invert ? header->actor0 : header->actor1;

                for (PxU32 k = 0; k < header->numContactData; k++)
                {
                    const ::omni::physx::ContactData& cdata = contactData[k];
                    PxU32 elementIdx = dstStartIndices[i] + dstContactCount[i];

                    if (elementIdx < getMaxContactDataCount())
                    {
                        dstContactCount[i]++;
                        if (!headerRef.invert)
                        {
                            dstForces[elementIdx] =
                                PxVec3(cdata.impulse.x, cdata.impulse.y, cdata.impulse.z).magnitude() * timeStepInv;
                        }
                        else
                        {
                            dstForces[elementIdx] =
                                -PxVec3(cdata.impulse.x, cdata.impulse.y, cdata.impulse.z).magnitude() * timeStepInv;
                        }
                        dstNormals[elementIdx] = PxVec3(cdata.normal.x, cdata.normal.y, cdata.normal.z);
                        dstPoints[elementIdx] = PxVec3(cdata.position.x, cdata.position.y, cdata.position.z);
                        dstSeparations[elementIdx] = cdata.separation;
                        dstActorIds[elementIdx * 2 + 0] = sensorActor;
                        dstActorIds[elementIdx * 2 + 1] = otherActor;
                    }
                }
            }
        }

        // A fully-truncated sensor keeps its first-pass prefix-sum start, which can exceed
        // the buffer. Clamp to capacity so `[start, start + count)` stays in range for every
        // sensor; count is already 0 there, so the slice is empty either way.
        {
            const PxU32 cap = getMaxContactDataCount();
            for (PxU32 i = 0; i < numSensors; i++)
            {
                if (dstStartIndices[i] > cap)
                    dstStartIndices[i] = cap;
            }
        }

        // Interleave the scratch into the caller's (numSensors, 2) layout tensor.
        for (PxU32 i = 0; i < numSensors; i++)
        {
            dstSensorLayout[i * 2 + 0] = dstContactCount[i];
            dstSensorLayout[i * 2 + 1] = dstStartIndices[i];
        }
    }

    return true;
}

}
}
}
