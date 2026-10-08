// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "Setup.h"
#include "usdLoad/LoadUsd.h"

#include <omni/physics/parse/Handles.h>

namespace omni
{
namespace physx
{

struct ObjectDataQueryType
{
    enum Enum
    {
        eOBJECT_ID,
        ePHYSX_PTR,
        eINTERNAL_PTR
    };
};

// ObjectKey-keyed lookup (ADR-0019). All former SdfPath-typed callers have been retyped
// to pass ObjectKey directly (see PhysXPropertyQuery.cpp's processArticulation and
// executeQueryRigidBody), so the SdfPath overload this once had is gone.
template <ObjectDataQueryType::Enum queryType>
size_t getObjectDataOrID(omni::physics::parse::ObjectKey key,
                         PhysXType type,
                         const internal::InternalPhysXDatabase& internalDatabase,
                         const omni::physx::usdparser::AttachedStage& attachedStage)
{
    if (key.valid())
    {
        const usdparser::ObjectIdMap* entries = attachedStage.getObjectIds(key);
        if (entries && !entries->empty())
        {
            auto it = entries->begin();
            while (it != entries->end())
            {
                usdparser::ObjectId objectId = it->second;

                if (queryType == ObjectDataQueryType::eOBJECT_ID)
                {
                    if (internalDatabase.checkRecordType(type, objectId))
                        return size_t(objectId);
                }
                else
                {
                    void* ptr;

                    if (queryType == ObjectDataQueryType::ePHYSX_PTR)
                        ptr = internalDatabase.getTypedRecord(type, objectId);
                    else
                        ptr = internalDatabase.getInternalTypedRecord(type, objectId);

                    if (ptr)
                    {
                        return size_t(ptr);
                    }
                }

                it++;
            }
        }
    }
    else
    {
        if ((queryType == ObjectDataQueryType::ePHYSX_PTR) && (type == ePTPhysics))
        {
            return size_t(omni::physx::OmniPhysX::getInstance().getPhysXSetup().getPhysics());
        }
    }

    if (queryType <= ObjectDataQueryType::eOBJECT_ID)
        return size_t(usdparser::kInvalidObjectId);
    else
        return size_t(NULL);
}

} // namespace physx
} // namespace omni
