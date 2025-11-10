// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include <UsdPCH.h>
// clang-format on

#include "CpuRigidContactView.h"
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

CpuRigidContactView::CpuRigidContactView(CpuSimulationView* sim,
                                         const std::vector<RigidContactSensorEntry>& entries,
                                         uint32_t numFilters,
                                         uint32_t maxContactDataCount)
    : BaseRigidContactView(sim, entries, numFilters, maxContactDataCount)
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

        //printf("--------------------------\n");

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
                        //printf("~!~! Sensor %u, filter %u %c(%f, %f, %f)\n", i, idx, (headerRef.invert ? '-' : '+'),
                        //    invDt * cdata.impulse.x, invDt * cdata.impulse.y, invDt * cdata.impulse.z);
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
}
}
}
