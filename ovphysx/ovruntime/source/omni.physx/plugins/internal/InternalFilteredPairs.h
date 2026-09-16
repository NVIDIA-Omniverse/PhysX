// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <common/foundation/Allocator.h>
#include <private/omni/physx/PhysxUsd.h>

namespace omni
{
namespace physx
{
namespace usdparser
{
class AttachedStage;
}

namespace internal
{


class InternalFilteredPairs : public Allocateable
{
public:
    InternalFilteredPairs()
    {
    }

    usdparser::ObjectIdPairVector mPairs;

    void createFilteredPairs();
    void removeFilteredPairs();
};

void changeFilteredPairs(omni::physx::usdparser::AttachedStage& attachedStage, omni::physics::parse::ObjectKey key, bool removed);

} // namespace internal
} // namespace physx
} // namespace omni
