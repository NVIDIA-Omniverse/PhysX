// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#include "UsdPCH.h"

#include "OmniPhysX.h"
#include "PhysXScene.h"
#include "PhysXPrivate.h"
#include "PhysXTools.h"

#include "ObjectDataQuery.h"

using namespace ::physx;

namespace omni
{
namespace physx
{

PxScene* privGetPhysXScene()
{
    // A.B. Multi scenes support
    PhysXScene* ps = OmniPhysX::getInstance().getPhysXSetup().getPhysXScene(0);

    if (ps != nullptr)
    {
        return ps->getScene();
    }

    return nullptr;
}

void primGetRigidBodyInstancedData(usdparser::ObjectId* ids, uint32_t numIds, InstancedData* dataArray)
{
    const internal::InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    for (uint32_t i = 0; i < numIds; i++)
    {
        const usdparser::ObjectId id = ids[i];
        InstancedData& data = dataArray[i];
        data.instanceIndex = internal::kInvalidUint32_t;
        data.instancerPath = 0;

        if (id < db.getRecords().size())
        {
            const internal::InternalDatabase::Record& record = db.getRecords()[id];
            if (record.mInternalPtr && record.mType == ePTActor)
            {
                internal::InternalActor* intActor = reinterpret_cast<internal::InternalActor*>(record.mInternalPtr);
                if (intActor->mInstanceIndex != internal::kInvalidUint32_t)
                {
                    data.instanceIndex = intActor->mInstanceIndex;
                    data.instancerPath = asInt(intActor->mInstancePrim.GetPrimPath());
                }
            }
        }
    }
}

PxCudaContextManager* privGetCudaContextManager()
{
    return OmniPhysX::getInstance().getPhysXSetup().getCudaContextManager();
}

} // namespace physx
} // namespace omni
