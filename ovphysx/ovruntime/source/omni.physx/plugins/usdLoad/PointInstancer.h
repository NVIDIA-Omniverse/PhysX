// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
/**
 * @implements REQ-PARSE-CONSUMER-001
 * @covers AC-7
 */

#pragma once

#include <private/omni/physx/PhysxUsd.h>
#include "LoadTools.h"

#include <memory>

namespace omni::physics::usd
{
class ScannedStage;
}

namespace omni
{
namespace physx
{
namespace usdparser
{
using ObjectIdVector = std::vector<ObjectId>;
using ShapeDescVector = std::vector<std::pair<PXR_NS::SdfPath, PhysxShapeDesc*>>;
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
    PXR_NS::SdfPath descPath;
    ShapeDescVector shapeDescVector;
    bool outsideInstancer;
    omni::physics::parse::Matrix4d protoTransformInverse;
    bool hasProtoTransformInverse;

    // Set by parsePrototype when driven via the parse-library scan
    // path.  Null when the legacy listener path is used (descs
    // managed by ICE_PLACEMENT_NEW / ICE_FREE in that case).
    std::unique_ptr<omni::physics::usd::ScannedStage> scannedStage;
};

using TargetDescVector = std::vector<TargetDesc>;


void parseRigidBodyInstancer(AttachedStage& attachedStage,
                             const PXR_NS::SdfPath& instancerPath,
                             CollisionPairVector& filteredPairs);

// True iff `primKey` is not under `instancerKey` in the source hierarchy
// (walks parents via IPhysicsSource).
bool isOutsideInstancer(const omni::physics::parse::IPhysicsSource* src,
                        omni::physics::parse::ObjectKey primKey,
                        omni::physics::parse::ObjectKey instancerKey);
} // namespace usdparser
} // namespace physx
} // namespace omni
