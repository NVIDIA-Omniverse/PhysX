// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "../PhysicsTypes.h"

#include <omni/physx/ContactEvent.h>

#include <list>
#include <memory>
#include <unordered_map>
#include <vector>

namespace omni
{
namespace physx
{
struct ContactEventHeader;
struct ContactData;

namespace tensors
{
class CpuRigidBodyView;
class SimulationBackend;

struct RigidBodyDirtyForceFlags
{
    enum
    {
        eForce = 1 << 0,
        eTorque = 1 << 1,
    };
};

struct CpuRigidBodyDirtyForceTracker
{
    std::vector<::physx::PxRigidBody*> bodies;
    std::vector<uint8_t> dirtyFlags;

    // whether any of the bodies have dirty forces or torques
    bool isDirty = false;
};

using CpuRigidBodyDirtyForceTrackerPtr = std::shared_ptr<CpuRigidBodyDirtyForceTracker>;

struct RigidContactHeaderRef
{
    const ::omni::physx::ContactEventHeader* header;
    bool invert;

    RigidContactHeaderRef() : header(nullptr), invert(false)
    {
    }

    RigidContactHeaderRef(const ::omni::physx::ContactEventHeader* header, bool invert) : header(header), invert(invert)
    {
    }
};

class RigidContactBucket
{
public:
    void addHeaderRef(const ::omni::physx::ContactEventHeader* header, bool invert)
    {
        mHeaderRefs.emplace_back(header, invert);
        mContactCount += header->numContactData;
    }

    uint32_t getContactCount() const
    {
        return mContactCount;
    }

    uint32_t getHeaderCount() const
    {
        return uint32_t(mHeaderRefs.size());
    }

    const RigidContactHeaderRef& getHeaderRef(uint32_t idx) const
    {
        return mHeaderRefs[idx];
    }

    void clear()
    {
        mHeaderRefs.clear();
        mContactCount = 0;
    }

private:
    std::vector<RigidContactHeaderRef> mHeaderRefs;
    uint32_t mContactCount = 0;
};


struct CpuSimulationData
{
    CpuSimulationData(SimulationBackend& backend, long stageId);
    ~CpuSimulationData();

    // needed so that body forces can be cleared even after the views get deleted
    void addRigidBodyDirtyForceTracker(CpuRigidBodyDirtyForceTrackerPtr tracker);

    void clearForces();

    void updateContactReports();
    void updateFrictiontReports();

    void addRigidContactBucket(uint64_t referentId, RigidContactBucket* bucket);
    void removeRigidContactBucket(uint64_t referentId, RigidContactBucket* bucket);

    const ::omni::physx::ContactData* getCurrentContactData() const
    {
        return mCurrentContactData;
    }
    const ::omni::physx::FrictionAnchor* getCurrentFrictionData() const
    {
        return mFrictionAnchorData;
    }

private:
    SimulationBackend& mBackend;

    long mStageId = -1;

    // we need to keep these trackers so that dirty forces can be cleared even if the RigidBodyView gets released
    std::list<CpuRigidBodyDirtyForceTrackerPtr> mRigidBodyDirtyForceTrackers;

    int64_t mRigidContactTimestamp = -1;
    int64_t mRigidFrictionContactTimestamp = -1;
    std::unordered_map<uint64_t, std::vector<RigidContactBucket*>> mRigidContactBuckets;

    const ::omni::physx::ContactData* mCurrentContactData = nullptr;
    const ::omni::physx::FrictionAnchor* mFrictionAnchorData = nullptr;
};

using CpuSimulationDataPtr = std::shared_ptr<CpuSimulationData>;

} // namespace tensors
} // namespace physx
} // namespace omni
