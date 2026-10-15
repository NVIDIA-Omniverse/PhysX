// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-ATTACH-IO-001
// @covers AC-2
// @implements REQ-CCT-CONTROL-001
// @covers AC-2

#pragma once

/**
 * @implements REQ-READ-CORE-001
 * @covers AC-7
 */

#include <atomic>

#include <common/foundation/Allocator.h>

#include <omni/physx/IPhysx.h>
#include <private/omni/physx/PhysxUsd.h>

#include <omni/physics/parse/Handles.h>

#include <vector>

namespace physx { class PxScene; }

namespace omni
{
namespace physx
{
namespace internal
{

// Read only the database owned by this scene. A peer's mutations do not
// invalidate the cached topology. Owner counters survive database rebuilding.
uint64_t recordLifetimeEpoch(const ::physx::PxScene* scene);
uint64_t kinematicToggleEpoch(const ::physx::PxScene* scene);

class InternalDatabase : public Allocateable
{
public:
    struct Record
    {
        Record()
        {
        }

        __forceinline Record(void* ptr, PhysXType type, void* internalPtr, omni::physics::parse::ObjectKey key,
                             std::atomic<uint64_t>* ownerEpoch = nullptr)
            : mPtr(ptr), mInternalPtr(internalPtr), mKey(key), mType(type), mOwnerEpoch(ownerEpoch),
              mGeneration(ownerEpoch ? ownerEpoch->load(std::memory_order_relaxed) : 0)
        {
        }
        void* mPtr = nullptr;
        void* mInternalPtr = nullptr;
        omni::physics::parse::ObjectKey mKey;
        PhysXType mType = ePTRemoved;
        std::atomic<uint64_t>* mOwnerEpoch = nullptr;
        // Captured before addRecord increments the owner epoch. Unlike the live
        // epoch this survives unrelated record edits; replacement gets a new value.
        uint64_t mGeneration = 0;

        void setRemoved()
        {
            mPtr = nullptr;
            mInternalPtr = nullptr;
            mType = ePTRemoved;
            if (mOwnerEpoch)
                mOwnerEpoch->fetch_add(1, std::memory_order_relaxed);
        }
    };

    explicit InternalDatabase(std::atomic<uint64_t>* ownerEpoch = nullptr);
    virtual ~InternalDatabase();
    InternalDatabase(const InternalDatabase&) = delete;
    InternalDatabase& operator=(const InternalDatabase&) = delete;
    uint64_t recordEpoch() const { return mRecordEpoch->load(std::memory_order_relaxed); }

    usdparser::ObjectId addRecord(PhysXType type, void* ptr, void* internalPtr, omni::physics::parse::ObjectKey key);
    usdparser::ObjectId addRecordAtIndex(
        size_t index, PhysXType type, void* ptr, void* internalPtr, omni::physics::parse::ObjectKey key);
    void* getTypedRecord(PhysXType type, usdparser::ObjectId) const;
    void* getRecord(PhysXType& type, usdparser::ObjectId) const;
    const Record* getFullTypedRecord(PhysXType type, usdparser::ObjectId) const;
    Record* getFullTypedRecord(PhysXType type, usdparser::ObjectId);
    const Record* getFullRecord(PhysXType& type, usdparser::ObjectId) const;
    Record* getFullRecord(PhysXType& type, usdparser::ObjectId);
    void* getInternalTypedRecord(PhysXType type, usdparser::ObjectId) const;
    bool checkRecordType(PhysXType type, usdparser::ObjectId) const;
    std::vector<Record>& getRecords()
    {
        return mRecords;
    }
    const std::vector<Record>& getRecords() const
    {
        return mRecords;
    }

protected:
private:
    std::atomic<uint64_t> mLocalEpoch{1};
    std::atomic<uint64_t>* mRecordEpoch;
    std::vector<Record> mRecords;
};
} // namespace internal
} // namespace physx
} // namespace omni
