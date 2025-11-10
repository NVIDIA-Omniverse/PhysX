// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

PhysxJointDesc* parseJoint(pxr::UsdStageWeakPtr stage,
                           const omni::physics::schema::JointDesc& inDesc,
                           pxr::UsdGeomXformCache& xfCache);

// Create joint desc from the given PhysicsJoint prim
// does not parse, just creates the correct joint desc type
PhysxJointDesc* createJointDesc(const pxr::UsdPrim& usdPrim);

ObjectId createJoint(AttachedStage& attachedStage,
                     const pxr::SdfPath& primPath,
                     omni::physx::usdparser::PhysxJointDesc* desc,
                     ObjectId body0,
                     bool body0Dynamic,
                     ObjectId body1,
                     bool body1Dynamic);

bool checkJointBodySimulationOwners(AttachedStage& attachedStage,
                                    pxr::UsdStageWeakPtr stage,
                                    const omni::physics::schema::JointDesc& inDesc);


} // namespace usdparser
} // namespace physx
} // namespace omni
