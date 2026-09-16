// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <private/omni/physx/PhysxUsd.h>

namespace omni
{
namespace physx
{
namespace usdparser
{

ObjectId createJoint(AttachedStage& attachedStage,
                     omni::physics::parse::ObjectKey primKey,
                     omni::physx::usdparser::PhysxJointDesc* desc,
                     ObjectId body0,
                     bool body0Dynamic,
                     ObjectId body1,
                     bool body1Dynamic);


} // namespace usdparser
} // namespace physx
} // namespace omni
