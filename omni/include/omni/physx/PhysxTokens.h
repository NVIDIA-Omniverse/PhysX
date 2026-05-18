// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

namespace omni
{

namespace physx
{

static const char gLocalMatrixTokenString[] = "omni:fabric:localMatrix";
static const char gWorldMatrixTokenString[] = "omni:fabric:worldMatrix";
static const char gWorldInstanceTransformTokenString[] = "omni:fabric:worldInstanceTransform";
static const char gResetXformStackTokenString[] = "omni::fabric::resetXformStack";
static const char gWorldVisibilityTokenString[] = "_worldVisibility";
static const char gLocalVisibilityTokenString[] = "visibility";

static const char gWorldForceTokenString[] = "_worldForce";
static const char gWorldTorqueTokenString[] = "_worldTorque";
static const char gPointsTokenString[] = "points";
static const char gInitPointsTokenString[] = "_initPoints";
static const char gDynamicBodyTokenString[] = "dynamicBody";
static const char gNestedBodyTokenString[] = "nestedBody";
static const char gRigidBodyAPITokenString[] = "PhysicsRigidBodyAPI";
static const char gPhysXPtrTokenString[] = "_physxPtr";
static const char gPhysXPtrInstancedTokenString[] = "_physxPtrInstanced";

static const char gPositionInvMassesTokenString[] = "_positionInvMasses";
static const char gVelocitiesFloat4TokenString[] = "_velocitiesFloat4";

static const char gRigidBodyWorldPositionTokenString[] = "_rigidBodyWorldPosition";
static const char gRigidBodyWorldOrientationTokenString[] = "_rigidBodyWorldOrientation";
static const char gRigidBodyWorldScaleTokenString[] = "_rigidBodyWorldScale";

} // namespace physx
} // namespace omni
