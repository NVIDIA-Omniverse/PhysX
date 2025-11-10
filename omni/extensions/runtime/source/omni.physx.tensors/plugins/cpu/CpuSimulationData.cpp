// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include <UsdPCH.h>
// clang-format on

#include "CpuSimulationData.h"
#include "../GlobalsAreBad.h"
#include "../SimulationBackend.h"

#include <omni/physx/IPhysxSimulation.h>

#include <PxRigidBody.h>

using namespace physx;

namespace omni
{
namespace physx
{
namespace tensors
{

CpuSimulationData::CpuSimulationData(SimulationBackend& backend, long stageId)
    : mBackend(backend)
    , mStageId(stageId)
{
}

CpuSimulationData::~CpuSimulationData()
{
    // printf("+++++ Deleting CpuSimulationData\n");
}

void CpuSimulationData::addRigidBodyDirtyForceTracker(CpuRigidBodyDirtyForceTrackerPtr tracker)
{
    mRigidBodyDirtyForceTrackers.push_back(tracker);
}

void CpuSimulationData::clearForces()
{
    for (auto it = mRigidBodyDirtyForceTrackers.begin(); it != mRigidBodyDirtyForceTrackers.end(); /*noop*/)
    {
        CpuRigidBodyDirtyForceTrackerPtr& tracker = *it;

        if (tracker->isDirty)
        {
            // printf("CLEARING RB FORCES\n");

            PxU32 numBodies = PxU32(tracker->bodies.size());
            for (PxU32 i = 0; i < numBodies; i++)
            {
                PxRigidBody* body = tracker->bodies[i];
                uint8_t& dirtyFlags = tracker->dirtyFlags[i];

                if (dirtyFlags & RigidBodyDirtyForceFlags::eForce)
                {
                    body->clearForce();
                }
                if (dirtyFlags & RigidBodyDirtyForceFlags::eTorque)
                {
                    body->clearTorque();
                }

                dirtyFlags = 0;
            }

            tracker->isDirty = false;
        }

        // if the corresponding RigidBodyView was deleted, we can remove this tracker
        if (tracker.use_count() == 1)
        {
            // printf("REMOVING RB FORCE TRACKER\n");
            it = mRigidBodyDirtyForceTrackers.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

void CpuSimulationData::addRigidContactBucket(uint64_t referentId, RigidContactBucket* bucket)
{
    mRigidContactBuckets[referentId].push_back(bucket);
}

void CpuSimulationData::removeRigidContactBucket(uint64_t referentId, RigidContactBucket* bucket)
{
    auto referentIter = mRigidContactBuckets.find(referentId);
    if (referentIter != mRigidContactBuckets.end())
    {
        auto& bucketList = referentIter->second;
        auto bucketIter = std::find(bucketList.begin(), bucketList.end(), bucket);
        if (bucketIter != bucketList.end())
        {
            bucketList.erase(bucketIter);
        }
        if (bucketList.empty())
        {
            mRigidContactBuckets.erase(referentIter);
        }
    }
}

void CpuSimulationData::updateContactReports()
{
    if (!g_physxSimulation)
    {
        return;
    }

    // avoid processing contacts multiple times during a step
    int64_t timestamp = mBackend.getTimestamp();
    if (mRigidContactTimestamp >= timestamp)
    {
        return;
    }
    mRigidContactTimestamp = timestamp;

    // clear existing contact buckets
    for (auto& referentItem : mRigidContactBuckets)
    {
        for (auto& bucket : referentItem.second)
        {
            bucket->clear();
        }
    }
    mCurrentContactData = nullptr;

    const ::omni::physx::ContactEventHeader* contactHeaders = nullptr;
    const ::omni::physx::ContactData* contactData = nullptr;
    const ::omni::physx::FrictionAnchor* frictionAnchorData = nullptr;
    PxU32 numContacts = 0;
    PxU32 numFrictionAnchorData = 0;
    PxU32 numHeaders = g_physxSimulation->getFullContactReport(&contactHeaders, &contactData, numContacts, &frictionAnchorData, numFrictionAnchorData);

    //printf("~!~!~! Got %u headers, %u contacts\n", numHeaders, numContacts);

    if (!numHeaders || !numContacts || !contactHeaders || !contactData)
    {
        return;
    }

    //
    // TODO: multi-thread this for large sims?
    //
    for (PxU32 i = 0; i < numHeaders; i++)
    {
        auto& header = contactHeaders[i];

#if 0
        printf("~!~!~!   Type %d: (%llu, %llu) (%llu, %llu)\n", int(header.type),
            (unsigned long long)header.actor0, (unsigned long long)header.collider0,
            (unsigned long long)header.actor1, (unsigned long long)header.collider1);

        for (PxU32 j = 0; j < header.numContactData; j++)
        {
            auto& cdata = contactData[header.contactDataOffset + j];
            auto& pos = cdata.position;
            auto& normal = cdata.normal;
            printf("~!~!~!     (%f, %f, %f) (%f, %f, %f)\n",
                pos.x, pos.y, pos.z, normal.x, normal.y, normal.z);
        }
#endif

        // skip contact lost events
        if (header.type == ContactEventType::eCONTACT_LOST)
        {
            continue;
        }

        auto referentIter0 = mRigidContactBuckets.find(header.actor0);
        if (referentIter0 == mRigidContactBuckets.end())
        {
            referentIter0 = mRigidContactBuckets.find(header.collider0);
        }

        auto referentIter1 = mRigidContactBuckets.find(header.actor1);
        if (referentIter1 == mRigidContactBuckets.end())
        {
            referentIter1 = mRigidContactBuckets.find(header.collider1);
        }

        if (referentIter0 != mRigidContactBuckets.end())
        {
            auto& bucketList = referentIter0->second;
            for (auto& bucket : bucketList)
            {
                bucket->addHeaderRef(&header, false);
            }
        }

        if (referentIter1 != mRigidContactBuckets.end())
        {
            auto& bucketList = referentIter1->second;
            for (auto& bucket : bucketList)
            {
                bucket->addHeaderRef(&header, true);
            }
        }
    }

    mCurrentContactData = contactData;
    mFrictionAnchorData = frictionAnchorData;
}

}
}
}
