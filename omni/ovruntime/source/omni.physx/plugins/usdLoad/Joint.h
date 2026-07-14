// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <private/omni/physx/PhysxUsd.h>

namespace omni
{
namespace physx
{
namespace usdparser
{

PhysxJointDesc* parseJoint(PXR_NS::UsdStageWeakPtr stage,
                           const omni::physics::schema::JointDesc& inDesc,
                           PXR_NS::UsdGeomXformCache& xfCache);

// Create joint desc from the given PhysicsJoint prim
// does not parse, just creates the correct joint desc type
PhysxJointDesc* createJointDesc(const PXR_NS::UsdPrim& usdPrim);

ObjectId createJoint(AttachedStage& attachedStage,
                     const PXR_NS::SdfPath& primPath,
                     omni::physx::usdparser::PhysxJointDesc* desc,
                     ObjectId body0,
                     bool body0Dynamic,
                     ObjectId body1,
                     bool body1Dynamic);

bool checkJointBodySimulationOwners(AttachedStage& attachedStage,
                                    PXR_NS::UsdStageWeakPtr stage,
                                    const omni::physics::schema::JointDesc& inDesc);


} // namespace usdparser
} // namespace physx
} // namespace omni
