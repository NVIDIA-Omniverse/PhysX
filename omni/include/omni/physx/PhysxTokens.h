// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

namespace omni
{

namespace physx
{

static const char gWorldMatrixTokenString[] = "omni:fabric:worldMatrix";

static const char gWorldForceTokenString[] = "_worldForce";
static const char gWorldTorqueTokenString[] = "_worldTorque";
static const char gPointsTokenString[] = "points";
static const char gInitPointsTokenString[] = "_initPoints";
static const char gDynamicBodyTokenString[] = "dynamicBody";
static const char gPhysXPtrTokenString[] = "_physxPtr";
static const char gPhysXPtrInstancedTokenString[] = "_physxPtrInstanced";

static const char gLocalMatrixTokenString[] = "omni:fabric:localMatrix";

static const char gPositionInvMassesTokenString[] = "_positionInvMasses";
static const char gVelocitiesFloat4TokenString[] = "_velocitiesFloat4";

static const char gRigidBodyWorldPositionTokenString[] = "_rigidBodyWorldPosition";
static const char gRigidBodyWorldOrientationTokenString[] = "_rigidBodyWorldOrientation";
static const char gRigidBodyWorldScaleTokenString[] = "_rigidBodyWorldScale";

// The following are custom attributes for features not yet added to the schema.
// To be removed on next dot release and added to physx schema.
static const char gMimicJointNaturalFrequencyAttributeName[3][64] = { "physxMimicJoint:rotX:naturalFrequency",
                                                                      "physxMimicJoint:rotY:naturalFrequency",
                                                                      "physxMimicJoint:rotZ:naturalFrequency" };
static const char gMimicJointDampingRatioAttributeName[3][64] = { "physxMimicJoint:rotX:dampingRatio",
                                                                  "physxMimicJoint:rotY:dampingRatio",
                                                                  "physxMimicJoint:rotZ:dampingRatio" };
static const pxr::TfToken gMimicJointNaturalFrequencyAttributeNameToken[3] = {
    pxr::TfToken(gMimicJointNaturalFrequencyAttributeName[0]),
    pxr::TfToken(gMimicJointNaturalFrequencyAttributeName[1]), pxr::TfToken(gMimicJointNaturalFrequencyAttributeName[2])
};
static const pxr::TfToken gMimicJointDampingRatioAttributeNameToken[3] = {
    pxr::TfToken(gMimicJointDampingRatioAttributeName[0]), pxr::TfToken(gMimicJointDampingRatioAttributeName[1]),
    pxr::TfToken(gMimicJointDampingRatioAttributeName[2])
};


} // namespace physx
} // namespace omni
