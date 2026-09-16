// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @implements REQ-READ-CORE-001
 * @covers AC-7
 */

#include <common/foundation/Allocator.h>

#include <omni/physx/IPhysx.h>
#include <private/omni/physx/PhysxUsd.h>

#include <omni/physics/parse/Handles.h>

#include <vector>

namespace omni
{
namespace physx
{
namespace internal
{

// Monotonic count of PhysX object creations and removals: anything derived from the database (a body
// set, a row list, a scene topology) stays valid only while this is unchanged, so a consumer can
// revalidate a cached snapshot with one integer compare. Process-wide rather than per-database, so
// the count never restarts and can never match a snapshot describing objects that no longer exist.
uint64_t recordLifetimeEpoch();
void bumpRecordLifetimeEpoch();

class InternalDatabase : public Allocateable
{
public:
    struct Record
    {
        Record()
        {
        }

        __forceinline Record(void* ptr, PhysXType type, void* internalPtr, omni::physics::parse::ObjectKey key)
            : mPtr(ptr), mInternalPtr(internalPtr), mKey(key), mType(type)
        {
        }
        void* mPtr;
        void* mInternalPtr;
        omni::physics::parse::ObjectKey mKey;
        PhysXType mType;

        void setRemoved()
        {
            mPtr = nullptr;
            mInternalPtr = nullptr;
            mType = ePTRemoved;
            bumpRecordLifetimeEpoch(); // the only path by which an object stops existing
        }
    };

    InternalDatabase();
    virtual ~InternalDatabase();

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
    std::vector<Record> mRecords;
};
} // namespace internal
} // namespace physx
} // namespace omni
