// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-CONSUMER-001
 * @covers AC-7
 */

#pragma once

#include <private/omni/physx/PhysxUsd.h>
#include "LoadTools.h"

#include <memory>

namespace omni::physics::parse
{
class IPhysicsSource;
class ScannedStage;
} // namespace omni::physics::parse

namespace omni
{
namespace physx
{
namespace usdparser
{

// ObjectKey-native (ADR-0018): TargetDesc owns a `omni::physics::parse::ScannedStage` (the
// backend-agnostic scan result, ScannedStage.h) rather than the USD-derived
// `omni::physics::usd::ScannedStage` -- the pxr-typed `pathFor()` resolver it used is not on the
// base class, so callers re-key through the source's string identity instead (see
// PointInstancer.cpp / ScannedShapeCookingDispatch.cpp's `sourceKeyToString` idiom). The engine-side
// object creation this pipeline drives (PhysXUsdPhysicsInterface::createObject/createShape) has
// ObjectKey overloads (usdInterface/UsdInterface.h), so this pipeline is pxr-free end to end.
using ObjectIdVector = std::vector<ObjectId>;
using ShapeDescVector = std::vector<std::pair<omni::physics::parse::ObjectKey, PhysxShapeDesc*>>;
struct TargetDesc
{
    TargetDesc();
    ~TargetDesc();
    // Move-only: scannedStage owns parse-library data referenced
    // (non-owning) by entries in shapeDescVector / desc.  Lifetime
    // contract: the ScannedStage outlives all uses of shape/body
    // descs derived from it, including the per-instance scale + cook
    // step in parseRigidBodyInstancer.
    TargetDesc(TargetDesc&&) noexcept;
    TargetDesc& operator=(TargetDesc&&) noexcept;
    TargetDesc(const TargetDesc&) = delete;
    TargetDesc& operator=(const TargetDesc&) = delete;

    PhysxObjectDesc* desc;
    omni::physics::parse::ObjectKey descKey;
    ShapeDescVector shapeDescVector;
    bool outsideInstancer;
    omni::physics::parse::Matrix4d protoTransformInverse;
    bool hasProtoTransformInverse;

    // Set by parsePrototype when driven via the parse-library scan
    // path.  Null when the legacy listener path is used (descs
    // managed by ICE_PLACEMENT_NEW / ICE_FREE in that case).
    std::unique_ptr<omni::physics::parse::ScannedStage> scannedStage;
};

using TargetDescVector = std::vector<TargetDesc>;


void parseRigidBodyInstancer(AttachedStage& attachedStage,
                             omni::physics::parse::ObjectKey instancerKey,
                             CollisionPairVector& filteredPairs);

// True iff `primKey` is not under `instancerKey` in the source hierarchy
// (walks parents via IPhysicsSource).
bool isOutsideInstancer(const omni::physics::parse::IPhysicsSource* src,
                        omni::physics::parse::ObjectKey primKey,
                        omni::physics::parse::ObjectKey instancerKey);
} // namespace usdparser
} // namespace physx
} // namespace omni
