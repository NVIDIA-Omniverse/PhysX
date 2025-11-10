// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <common/foundation/Allocator.h>

#include "UsdPCH.h"
#include <omni/physx/IPhysx.h>
#include <private/omni/physx/PhysxUsd.h>

#include <vector>

namespace omni
{
namespace physx
{
namespace internal
{

class InternalDatabase : public Allocateable
{
public:
    struct Record
    {
        Record()
        {
        }

        __forceinline Record(void* ptr, PhysXType type, void* internalPtr, const pxr::SdfPath& path)
            : mPtr(ptr), mInternalPtr(internalPtr), mPath(path), mType(type)
        {
        }
        void* mPtr;
        void* mInternalPtr;
        pxr::SdfPath mPath;
        PhysXType mType;

        void setRemoved()
        {
            mPtr = nullptr;
            mInternalPtr = nullptr;
            mType = ePTRemoved;
        }
    };

    InternalDatabase();
    virtual ~InternalDatabase();

    usdparser::ObjectId addRecord(PhysXType type, void* ptr, void* internalPtr, const pxr::SdfPath& path);
    usdparser::ObjectId addRecordAtIndex(
        size_t index, PhysXType type, void* ptr, void* internalPtr, const pxr::SdfPath& path);
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
