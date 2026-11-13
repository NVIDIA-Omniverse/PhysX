// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-PUBLICAPI-001
// @covers AC-51

#pragma once
#include <carb/Types.h>
#include <cstdint>
#include <vector>

namespace omni
{

namespace physx
{
struct PhysxPropertyQueryResult
{
    enum Enum
    {
        eVALID = 0,
        eERROR_UNKNOWN_QUERY_MODE = 1,
        eERROR_INVALID_USD_PATH = 2,
        eERROR_INVALID_USD_STAGE = 3,
        eERROR_INVALID_USD_PRIM = 4,
        eERROR_PARSING = 5,
        eERROR_TIMEOUT = 6,
        eERROR_RUNTIME = 7,
    };
};

struct PhysxPropertyQueryBaseResponse
{
    PhysxPropertyQueryResult::Enum result = PhysxPropertyQueryResult::eVALID;
    uint64_t usdStageId = { 0 };
    uint64_t usdPath = { 0 };
};

struct PhysxPropertyQueryRigidBodyType
{
    enum Enum
    {
        eTYPE_RIGID_DYNAMIC = 0
    };
};

struct PhysxPropertyQueryRigidBodyResponse : public PhysxPropertyQueryBaseResponse
{
    PhysxPropertyQueryRigidBodyType::Enum type = PhysxPropertyQueryRigidBodyType::eTYPE_RIGID_DYNAMIC;
    float mass = 0;
    carb::Float3 inertia = { 0, 0, 0 };
    carb::Float3 centerOfMass = { 0, 0, 0 };
    carb::Float4 principalAxes = { 0, 0, 0, 1 };
};

struct PhysxPropertyQueryColliderResponse : public PhysxPropertyQueryBaseResponse
{
    carb::Float3 aabbLocalMin = { 0, 0, 0 };
    carb::Float3 aabbLocalMax = { 0, 0, 0 };
    float volume = 0;
    carb::Float3 localPos = { 0, 0, 0 };
    carb::Float4 localRot = { 0, 0, 0, 1 };
};

struct PhysxPropertyQueryArticulationLink
{
    uint64_t rigidBody;
    uint64_t joint;
    uint32_t jointDof;
};

struct PhysxPropertyQueryArticulationResponse : public PhysxPropertyQueryBaseResponse
{
    std::vector<PhysxPropertyQueryArticulationLink> links;
};


typedef void (*PhysxPropertyQueryColliderCallback)(const PhysxPropertyQueryColliderResponse&, void*);
typedef void (*PhysxPropertyQueryRigidBodyCallback)(const PhysxPropertyQueryRigidBodyResponse&, void*);
typedef void (*PhysxPropertyQueryArticulationCallback)(const PhysxPropertyQueryArticulationResponse&, void*);
typedef void (*PhysxPropertyQueryFinishedCallback)(void*);

struct PhysxPropertyQueryMode
{
    enum Enum
    {
        eQUERY_RIGID_BODY_WITH_COLLIDERS = 0,
        eQUERY_ARTICULATION = 1
    };
};

struct PhysxPropertyQueryCallbacks
{
    void* userData = { nullptr };
    int64_t timeoutMs = -1; // -1 means infinite timeout
    PhysxPropertyQueryFinishedCallback queryFinishedCallback = { nullptr }; // Called when enumeration is finished
    PhysxPropertyQueryRigidBodyCallback rigidBodyCallback = { nullptr }; // Called when rigid body is found
    PhysxPropertyQueryColliderCallback colliderCallback = { nullptr }; // Called when collider is found
    PhysxPropertyQueryArticulationCallback articulationCallback = { nullptr }; // Called when articulation is found
};

} // namespace physx
} // namespace omni
