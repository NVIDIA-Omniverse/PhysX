// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

// PhysX Compound Shape definition, helper class
namespace omni
{
namespace physx
{
class PhysXCompoundShape
{
public:
    PhysXCompoundShape()
    {
    }

    ~PhysXCompoundShape()
    {
    }

    const std::vector<::physx::PxShape*>& getShapes() const
    {
        return mShapes;
    }

private:
    std::vector<::physx::PxShape*> mShapes;
};
} // namespace physx
} // namespace omni
