// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-44
 */

#include "OmniPhysX.h"
#include "PhysXScene.h"
#include "PhysXPrivate.h"
#include "PhysXTools.h"

#include "ObjectDataQuery.h"

#include <usdLoad/LoadUsd.h>

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
                    // instancerPath is now
                    // mInstanceKey.handle directly (the ENCODE direction -- see IPhysxPrivate.h's
                    // updated doc comment), not asInt(as->pathFor(...)). No AttachedStage lookup
                    // needed any more, so this is unconditional -- and it no longer depends on
                    // getActiveAttachedStage() happening to be the SPECIFIC attach that owns
                    // intActor (a latent gap in the old code, which used "whichever attach is
                    // active" rather than the actual owner). Generation-tag caveat (ADR-0021): a
                    // consumer that reads this across a detach/reattach of the owning attach gets
                    // a handle from the old generation, which will not resolve against the new
                    // one -- a resolution failure on the consumer's side, not decode garbage.
                    data.instancerPath = intActor->mInstanceKey.handle;
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
