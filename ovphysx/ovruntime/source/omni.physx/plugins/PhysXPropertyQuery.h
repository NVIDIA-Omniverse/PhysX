// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "UsdPCH.h"

#include <omni/physx/IPhysxPropertyQuery.h>

#include <carb/extras/Timer.h>


namespace omni
{
namespace kit
{
struct StageUpdateNode;
struct StageUpdateSettings;
} // namespace kit
namespace physx
{
class PhysXPropertyQueryManager;
void queryPrim(uint64_t stageId,
               uint64_t primKey,
               PhysxPropertyQueryMode::Enum queryMode,
               const IPhysxPropertyQueryCallback& callbacks);

struct BaseRequest
{
    carb::extras::Timer timeoutTimer;

    PXR_NS::UsdStageWeakPtr stage;
    uint64_t stageId = 0;
    PXR_NS::SdfPath primKey;
    uint64_t primPathId = 0;
    IPhysxPropertyQueryCallback callbacks;
    virtual ~BaseRequest()
    {
    }
    bool isExpired();
    void callQueryFinished();
    void returnWithError(PhysxPropertyQueryResult::Enum queryResult);
    virtual void releaseParsedData()
    {
    }
    virtual void sendRigidBodyResponse(const PhysxPropertyQueryRigidBodyResponse&)
    {
    }
    virtual void sendColliderResponse(const PhysxPropertyQueryColliderResponse&)
    {
    }
};

struct RigidBodyRequest : public BaseRequest
{
    omni::physx::usdparser::PhysxRigidBodyDesc* desc = { nullptr };
    std::vector<std::pair<PXR_NS::SdfPath, omni::physx::usdparser::PhysxShapeDesc*>> colliders;

    virtual void releaseParsedData() override;
    virtual void sendRigidBodyResponse(const PhysxPropertyQueryRigidBodyResponse& response) override;
    virtual void sendColliderResponse(const PhysxPropertyQueryColliderResponse& response) override;
};

struct ArticulationRequest : public BaseRequest
{
    void sendArticulationResponse(const PhysxPropertyQueryArticulationResponse& response);
};


class PhysXPropertyQueryManager
{
    std::vector<RigidBodyRequest> rigidBodyRequests;

    bool allCollidersHaveBeenComputed(const RigidBodyRequest& rigidBodyRequest,
                                      PhysxPropertyQueryResult::Enum& errorCode);
    bool rigidBodyRequestHasFinished(RigidBodyRequest& rigidBodyRequest);
    void attachToStageUpdate();
    void detachFromStageUpdate();
    void executeQueryRigidBody(RigidBodyRequest& request);
    void executeQueryArticulation(ArticulationRequest& request);
    void processArticulation(const usdparser::AttachedStage& attachedStage, ArticulationRequest& request);

public:
    void queryPrim(uint64_t stageId,
                   uint64_t primPathId,
                   PhysxPropertyQueryMode::Enum queryMode,
                   const IPhysxPropertyQueryCallback& callbacks);

    void updateQueuedRequests();

    // Fail all queued rigid-body requests with an error and clear the queue.
    // Called on stage detach so that pending requests don't leak across tests
    // (and don't hold dangling UsdStageWeakPtrs that can crash later).
    void cancelAllPendingRequests(PhysxPropertyQueryResult::Enum errorCode);
};

} // namespace physx
} // namespace omni
