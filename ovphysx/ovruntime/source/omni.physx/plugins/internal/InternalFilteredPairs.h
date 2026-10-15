// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-ATTACH-RESOLVE-001
// @covers AC-5 AC-6

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
    explicit InternalFilteredPairs(usdparser::AttachedStage& attachedStage) : mAttachedStage(attachedStage) {}

    usdparser::ObjectIdPairVector mPairs;

    void createFilteredPairs();
    void removeFilteredPairs();

private:
    usdparser::AttachedStage& mAttachedStage;
};

void changeFilteredPairs(omni::physx::usdparser::AttachedStage& attachedStage, omni::physics::parse::ObjectKey key, bool removed);

} // namespace internal
} // namespace physx
} // namespace omni
