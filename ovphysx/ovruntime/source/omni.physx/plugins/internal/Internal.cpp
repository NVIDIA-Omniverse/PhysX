// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "Internal.h"

#include "stdint.h"

#include <atomic>

using namespace omni::physx;
using namespace omni::physx::internal;
using namespace omni::physx::usdparser;

namespace
{
// Relaxed ordering is enough: the value is only ever compared for equality against one a consumer
// read earlier, never used to publish other memory. Readers hold the same locks the mutations do.
std::atomic<uint64_t> gRecordLifetimeEpoch{ 1 };
} // namespace

namespace omni { namespace physx { namespace internal {

uint64_t recordLifetimeEpoch()
{
    return gRecordLifetimeEpoch.load(std::memory_order_relaxed);
}

void bumpRecordLifetimeEpoch()
{
    gRecordLifetimeEpoch.fetch_add(1, std::memory_order_relaxed);
}

}}} // namespace omni::physx::internal

InternalDatabase::InternalDatabase() = default;

InternalDatabase::~InternalDatabase() = default;

ObjectId InternalDatabase::addRecord(PhysXType type, void* ptr, void* internalPtr, omni::physics::parse::ObjectKey key)
{
    const uint32_t index = uint32_t(mRecords.size());
    mRecords.push_back(Record(ptr, type, internalPtr, key));
    bumpRecordLifetimeEpoch();
    return index;
}

ObjectId InternalDatabase::addRecordAtIndex(size_t index, PhysXType type, void* ptr, void* internalPtr, omni::physics::parse::ObjectKey key)
{
    mRecords[index] = Record(ptr, type, internalPtr, key);
    bumpRecordLifetimeEpoch();
    return index;
}

void* InternalDatabase::getTypedRecord(PhysXType type, ObjectId objectId) const
{
    const Record* record = getFullTypedRecord(type, objectId);
    if (record)
        return record->mPtr;
    else
        return nullptr;
}

void* InternalDatabase::getRecord(PhysXType& type, ObjectId objectId) const
{
    const uint32_t nbRecords = uint32_t(mRecords.size());
    if (objectId >= nbRecords)
        return nullptr;
    type = mRecords[objectId].mType;
    return mRecords[objectId].mPtr;
}

const InternalDatabase::Record* InternalDatabase::getFullTypedRecord(PhysXType type, ObjectId objectId) const
{
    const uint32_t nbRecords = uint32_t(mRecords.size());
    if (objectId >= nbRecords)
        return nullptr;
    if (mRecords[objectId].mType != type)
        return nullptr;
    return &mRecords[objectId];
}

InternalDatabase::Record* InternalDatabase::getFullTypedRecord(PhysXType type, ObjectId objectId)
{
    const uint32_t nbRecords = uint32_t(mRecords.size());
    if (objectId >= nbRecords)
        return nullptr;
    if (mRecords[objectId].mType != type)
        return nullptr;
    return &mRecords[objectId];
}

const InternalDatabase::Record* InternalDatabase::getFullRecord(PhysXType& type, ObjectId objectId) const
{
    const uint32_t nbRecords = uint32_t(mRecords.size());
    if (objectId >= nbRecords)
        return nullptr;
    type = mRecords[objectId].mType;        
    return &mRecords[objectId];
}

InternalDatabase::Record* InternalDatabase::getFullRecord(PhysXType& type, ObjectId objectId)
{
    const uint32_t nbRecords = uint32_t(mRecords.size());
    if (objectId >= nbRecords)
        return nullptr;
    type = mRecords[objectId].mType;
    return &mRecords[objectId];
}

void* InternalDatabase::getInternalTypedRecord(PhysXType type, ObjectId objectId) const
{
    const Record* record = getFullTypedRecord(type, objectId);
    if (record)
        return record->mInternalPtr;
    else
        return nullptr;
}

bool InternalDatabase::checkRecordType(PhysXType type, usdparser::ObjectId objectId) const
{
    const uint32_t nbRecords = uint32_t(mRecords.size());
    if (objectId >= nbRecords)
        return false;
    return (type == mRecords[objectId].mType);
}
