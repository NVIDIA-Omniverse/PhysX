// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-ATTACH-IO-001
// @covers AC-2

#include "Internal.h"
#include "OmniPhysX.h"
#include "PhysXScene.h"
#include "usdLoad/AttachedStage.h"

#include "stdint.h"

#include <atomic>

using namespace omni::physx;
using namespace omni::physx::internal;
using namespace omni::physx::usdparser;

namespace omni::physx::internal
{
uint64_t recordLifetimeEpoch(const ::physx::PxScene* scene)
{
    const auto* native = OmniPhysX::getInstance().getPhysXSetup().findPhysXScene(scene);
    return native ? native->getAttachedStage().recordVersion.load(std::memory_order_relaxed) : 0;
}
uint64_t kinematicToggleEpoch(const ::physx::PxScene* scene)
{
    const auto* native = OmniPhysX::getInstance().getPhysXSetup().findPhysXScene(scene);
    return native ? native->getAttachedStage().kinematicVersion.load(std::memory_order_relaxed) : 0;
}
}

InternalDatabase::InternalDatabase(std::atomic<uint64_t>* ownerEpoch)
    : mRecordEpoch(ownerEpoch ? ownerEpoch : &mLocalEpoch)
{
    mRecordEpoch->fetch_add(1, std::memory_order_relaxed);
}

InternalDatabase::~InternalDatabase() = default;

ObjectId InternalDatabase::addRecord(PhysXType type, void* ptr, void* internalPtr, omni::physics::parse::ObjectKey key)
{
    const uint32_t index = uint32_t(mRecords.size());
    mRecords.push_back(Record(ptr, type, internalPtr, key, mRecordEpoch));
    mRecordEpoch->fetch_add(1, std::memory_order_relaxed);
    return index;
}

ObjectId InternalDatabase::addRecordAtIndex(size_t index, PhysXType type, void* ptr, void* internalPtr, omni::physics::parse::ObjectKey key)
{
    mRecords[index] = Record(ptr, type, internalPtr, key, mRecordEpoch);
    mRecordEpoch->fetch_add(1, std::memory_order_relaxed);
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
