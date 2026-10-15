// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2
 */
#pragma once

#include "Descriptors.h"
#include "PreparedAttribute.h"
#include "KnownTokens.h"
#include <array>

namespace omni::physics::parse
{
class ParseContext;

struct PreparedDynamicBody
{
    bool prepared = false;
    SourceUnits units;
    bool outputVelocitiesLocalSpaceDefault = false;
    bool physxRigidBodyAPI = false;
    bool physxSurfaceVelocityAPI = false;
    bool physxSplinesSurfaceVelocityAPI = false;
    Matrix3d rotation;
    carb::Float3 scale{ 1.0f, 1.0f, 1.0f };
    ObjectKey splineCurve;
    PreparedAttribute<bool> physicsKinematicEnabled;
    PreparedAttribute<bool> physicsStartsAsleep;
    PreparedAttribute<carb::Float3> physicsVelocity;
    PreparedAttribute<carb::Float3> physicsAngularVelocity;
    PreparedAttribute<float> physxRigidBodyLinearDamping;
    PreparedAttribute<float> physxRigidBodyAngularDamping;
    PreparedAttribute<float> physxRigidBodyMaxLinearVelocity;
    PreparedAttribute<float> physxRigidBodySleepThreshold;
    PreparedAttribute<float> physxRigidBodyStabilizationThreshold;
    PreparedAttribute<float> physxRigidBodyMaxDepenetrationVelocity;
    PreparedAttribute<float> physxRigidBodyContactSlopCoefficient;
    PreparedAttribute<float> physxRigidBodyMaxContactImpulse;
    PreparedAttribute<float> physxRigidBodyCfmScale;
    PreparedAttribute<int> physxRigidBodySolverPositionIterationCount;
    PreparedAttribute<int> physxRigidBodySolverVelocityIterationCount;
    PreparedAttribute<bool> physxRigidBodyEnableCCD;
    PreparedAttribute<bool> physxRigidBodyEnableSpeculativeCCD;
    PreparedAttribute<bool> physxRigidBodyDisableGravity;
    PreparedAttribute<bool> physxRigidBodyRetainAccelerations;
    PreparedAttribute<bool> physxRigidBodyEnableGyroscopicForces;
    PreparedAttribute<bool> physxRigidBodySolveContact;
    PreparedAttribute<int> physxRigidBodyLockedPosAxis;
    PreparedAttribute<int> physxRigidBodyLockedRotAxis;
    PreparedAttribute<bool> physxSurfaceVelocityEnabled;
    PreparedAttribute<bool> physxSurfaceVelocityLocalSpace;
    PreparedAttribute<carb::Float3> physxSurfaceVelocity;
    PreparedAttribute<carb::Float3> physxSurfaceAngularVelocity;
    PreparedAttribute<bool> physxSplinesSurfaceVelocityEnabled;
    PreparedAttribute<float> physxSplinesSurfaceVelocityMagnitude;
    PreparedAttribute<bool> metadataLocalSpaceVelocities;
    PreparedAttribute<float> physxRigidBodyMaxAngularVelocity;
};

// The same attribute semantics serve a live serial source or an immutable
// ovstage scalar-row view. The latter decodes columns inside the worker.
template <typename Reader>
void readDynamicBodyAttributes(const Reader& src, ObjectKey key, const KnownTokens& tok,
                               PreparedDynamicBody& input)
{
    input.physicsKinematicEnabled = prepareAttribute<bool>(src, key, tok.physicsKinematicEnabled);
    input.physicsStartsAsleep = prepareAttribute<bool>(src, key, tok.physicsStartsAsleep);
    input.physicsVelocity = prepareAttribute<carb::Float3>(src, key, tok.physicsVelocity);
    input.physicsAngularVelocity = prepareAttribute<carb::Float3>(src, key, tok.physicsAngularVelocity);
    input.metadataLocalSpaceVelocities = prepareAttribute<bool>(src, key, tok.metadataLocalSpaceVelocities);
    if (input.physxRigidBodyAPI)
    {
        input.physxRigidBodyLinearDamping = prepareAttribute<float>(src, key, tok.physxRigidBodyLinearDamping, true);
        input.physxRigidBodyAngularDamping = prepareAttribute<float>(src, key, tok.physxRigidBodyAngularDamping, true);
        input.physxRigidBodyMaxLinearVelocity = prepareAttribute<float>(src, key, tok.physxRigidBodyMaxLinearVelocity, true);
        input.physxRigidBodySleepThreshold = prepareAttribute<float>(src, key, tok.physxRigidBodySleepThreshold, true);
        input.physxRigidBodyStabilizationThreshold = prepareAttribute<float>(src, key, tok.physxRigidBodyStabilizationThreshold, true);
        input.physxRigidBodyMaxDepenetrationVelocity = prepareAttribute<float>(src, key, tok.physxRigidBodyMaxDepenetrationVelocity, true);
        input.physxRigidBodyContactSlopCoefficient = prepareAttribute<float>(src, key, tok.physxRigidBodyContactSlopCoefficient, true);
        input.physxRigidBodyMaxContactImpulse = prepareAttribute<float>(src, key, tok.physxRigidBodyMaxContactImpulse, true);
        input.physxRigidBodyCfmScale = prepareAttribute<float>(src, key, tok.physxRigidBodyCfmScale, true);
        input.physxRigidBodySolverPositionIterationCount = prepareAttribute<int>(src, key, tok.physxRigidBodySolverPositionIterationCount, true);
        input.physxRigidBodySolverVelocityIterationCount = prepareAttribute<int>(src, key, tok.physxRigidBodySolverVelocityIterationCount, true);
        input.physxRigidBodyEnableCCD = prepareAttribute<bool>(src, key, tok.physxRigidBodyEnableCCD);
        input.physxRigidBodyEnableSpeculativeCCD = prepareAttribute<bool>(src, key, tok.physxRigidBodyEnableSpeculativeCCD);
        input.physxRigidBodyDisableGravity = prepareAttribute<bool>(src, key, tok.physxRigidBodyDisableGravity);
        input.physxRigidBodyRetainAccelerations = prepareAttribute<bool>(src, key, tok.physxRigidBodyRetainAccelerations);
        input.physxRigidBodyEnableGyroscopicForces = prepareAttribute<bool>(src, key, tok.physxRigidBodyEnableGyroscopicForces);
        input.physxRigidBodySolveContact = prepareAttribute<bool>(src, key, tok.physxRigidBodySolveContact);
        input.physxRigidBodyLockedPosAxis = prepareAttribute<int>(src, key, tok.physxRigidBodyLockedPosAxis, true);
        input.physxRigidBodyLockedRotAxis = prepareAttribute<int>(src, key, tok.physxRigidBodyLockedRotAxis, true);
        input.physxRigidBodyMaxAngularVelocity = prepareAttribute<float>(src, key, tok.physxRigidBodyMaxAngularVelocity);
    }
    if (input.physxSurfaceVelocityAPI)
    {
        input.physxSurfaceVelocityEnabled = prepareAttribute<bool>(src, key, tok.physxSurfaceVelocityEnabled);
        input.physxSurfaceVelocityLocalSpace = prepareAttribute<bool>(src, key, tok.physxSurfaceVelocityLocalSpace);
        input.physxSurfaceVelocity = prepareAttribute<carb::Float3>(src, key, tok.physxSurfaceVelocity);
        input.physxSurfaceAngularVelocity = prepareAttribute<carb::Float3>(src, key, tok.physxSurfaceAngularVelocity);
    }
    if (input.physxSplinesSurfaceVelocityAPI)
    {
        input.physxSplinesSurfaceVelocityEnabled = prepareAttribute<bool>(src, key, tok.physxSplinesSurfaceVelocityEnabled);
        input.physxSplinesSurfaceVelocityMagnitude = prepareAttribute<float>(src, key, tok.physxSplinesSurfaceVelocityMagnitude);
    }
}

inline std::array<TokenId, 31> dynamicBodyAttributeTokens(const KnownTokens& tok)
{
    return { tok.physicsKinematicEnabled, tok.physicsStartsAsleep, tok.physicsVelocity, tok.physicsAngularVelocity, tok.metadataLocalSpaceVelocities, tok.physxRigidBodyLinearDamping, tok.physxRigidBodyAngularDamping, tok.physxRigidBodyMaxLinearVelocity, tok.physxRigidBodySleepThreshold, tok.physxRigidBodyStabilizationThreshold, tok.physxRigidBodyMaxDepenetrationVelocity, tok.physxRigidBodyContactSlopCoefficient, tok.physxRigidBodyMaxContactImpulse, tok.physxRigidBodyCfmScale, tok.physxRigidBodySolverPositionIterationCount, tok.physxRigidBodySolverVelocityIterationCount, tok.physxRigidBodyEnableCCD, tok.physxRigidBodyEnableSpeculativeCCD, tok.physxRigidBodyDisableGravity, tok.physxRigidBodyRetainAccelerations, tok.physxRigidBodyEnableGyroscopicForces, tok.physxRigidBodySolveContact, tok.physxRigidBodyLockedPosAxis, tok.physxRigidBodyLockedRotAxis, tok.physxRigidBodyMaxAngularVelocity, tok.physxSurfaceVelocityEnabled, tok.physxSurfaceVelocityLocalSpace, tok.physxSurfaceVelocity, tok.physxSurfaceAngularVelocity, tok.physxSplinesSurfaceVelocityEnabled, tok.physxSplinesSurfaceVelocityMagnitude };
}

PreparedDynamicBody prepareDynamicBody(ParseContext& context, ObjectKey key);
bool fillDynamicBody(const PreparedDynamicBody& input, DynamicPhysxRigidBodyDesc& output) noexcept;

} // namespace omni::physics::parse
