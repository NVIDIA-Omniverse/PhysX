// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "Internal.h"

#include "stdint.h"

using namespace omni::physx;
using namespace omni::physx::internal;
using namespace omni::physx::usdparser;

InternalDatabase::InternalDatabase() = default;

InternalDatabase::~InternalDatabase() = default;

ObjectId InternalDatabase::addRecord(PhysXType type, void* ptr, void* internalPtr, const pxr::SdfPath& path)
{
    const uint32_t index = uint32_t(mRecords.size());
    mRecords.push_back(Record(ptr, type, internalPtr, path));
    return index;
}

ObjectId InternalDatabase::addRecordAtIndex(size_t index, PhysXType type, void* ptr, void* internalPtr, const pxr::SdfPath& path)
{    
    mRecords[index] = Record(ptr, type, internalPtr, path);
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
