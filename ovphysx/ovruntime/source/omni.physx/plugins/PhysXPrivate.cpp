// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-ATTACH-EXECUTION-001
// @covers AC-1 AC-2

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-44
 */

#include "OmniPhysX.h"
#include "PhysXScene.h"
#include "PhysXPrivate.h"
#include "PhysXTools.h"
#include "RuntimeExecution.h"

#include "ObjectDataQuery.h"

#include <usdLoad/LoadUsd.h>

using namespace ::physx;

namespace omni
{
namespace physx
{

PxScene* privGetPhysXScene(AttachHandle attach)
{
    RuntimeExecutionScope operation(attach);
    const usdparser::AttachedStage* owner = operation ? usdparser::UsdLoad::getUsdLoad()->getAttachedStageByHandle(attach) : nullptr;
    PhysXScene* ps = owner ? owner->getDefaultPhysXScene() : nullptr;

    if (ps != nullptr)
    {
        return ps->getScene();
    }

    return nullptr;
}

void primGetRigidBodyInstancedData(AttachHandle attach, usdparser::ObjectId* ids, uint32_t numIds, InstancedData* dataArray)
{
    if (!ids || !dataArray)
        return;
    RuntimeExecutionScope operation(attach);
    const usdparser::AttachedStage* owner = operation ? usdparser::UsdLoad::getUsdLoad()->getAttachedStageByHandle(attach) : nullptr;
    const internal::InternalPhysXDatabase* database = owner ? &owner->getInternalPhysXDatabase() : nullptr;

    for (uint32_t i = 0; i < numIds; i++)
    {
        const usdparser::ObjectId id = ids[i];
        InstancedData& data = dataArray[i];
        data.instanceIndex = internal::kInvalidUint32_t;
        data.instancerPath = 0;

        if (database && id < database->getRecords().size())
        {
            const internal::InternalDatabase::Record& record = database->getRecords()[id];
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

PxCudaContextManager* privGetCudaContextManager(AttachHandle attach, uint64_t sceneKey)
{
    RuntimeExecutionScope operation(attach, true);
    if (!operation)
        return nullptr;
    const std::shared_ptr<usdparser::AttachedStage> owner = usdparser::UsdLoad::getUsdLoad()->acquireAttachment(attach);
    if (!owner)
        return nullptr;
    for (const usdparser::AttachedStage::SceneMap::value_type& entry : owner->getPhysXScenes())
        if (entry.second->getSceneSdfPath().handle == sceneKey)
            return entry.second->getScene()->getCudaContextManager();
    return nullptr;
}

} // namespace physx
} // namespace omni
