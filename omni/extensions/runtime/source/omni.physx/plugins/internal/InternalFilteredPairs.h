// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

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

void changeFilteredPairs(omni::physx::usdparser::AttachedStage& attachedStage, const pxr::SdfPath& path, bool removed);

} // namespace internal
} // namespace physx
} // namespace omni
